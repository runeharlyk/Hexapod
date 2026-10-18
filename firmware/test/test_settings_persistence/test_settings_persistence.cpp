// Host tests of FSPersistencePB (template/stateful_persistence.h) on the host's file system: a save is written
// beside the stored file and moved over it, so a reset or a refused move during a save keeps the stored settings.

#include <unity.h>

#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <string>
#include <vector>

#include <template/stateful_persistence.h>
#include <settings/peripherals_settings.h>

namespace {

// What the next move of a written file over its destination does.
enum class Move { MOVES, FAILS, CUTS_POWER };
Move nextMove = Move::MOVES;
int movesCut = 0;

// Stands for the board resetting: nothing after the throw runs, and the files stay as the throw finds them.
struct PowerCut {};

} // namespace

namespace FileSystem {

bool mkdirRecursive(const char *path) {
    std::error_code error;
    std::filesystem::create_directories(path, error);
    return !error;
}

// Moves over an existing destination, as LittleFS does; the Windows C runtime's rename refuses to.
bool replaceFile(const char *from, const char *to) {
    if (nextMove == Move::FAILS) return false;
    if (nextMove == Move::CUTS_POWER) {
        ++movesCut;
        throw PowerCut {};
    }
    std::error_code error;
    std::filesystem::rename(from, to, error);
    return !error;
}

} // namespace FileSystem

namespace {

const std::string DIRECTORY = ".pio/test_settings_persistence";
const std::string FILE_PATH = DIRECTORY + "/config/peripheralSettings.json";
const std::string TEMPORARY_PATH = FILE_PATH + ".tmp";
const std::string BACKUP_PATH = FILE_PATH + ".bak";

// The peripheral settings service's persistence, with its real reader, validator and defaults.
struct Robot {
    StatefulService<PeripheralSettings> service {PeripheralSettings_defaults()};
    FSPersistencePB<PeripheralSettings> persistence {PeripheralSettings_read,
                                                     PeripheralSettings_update,
                                                     &service,
                                                     FILE_PATH.c_str(),
                                                     api_PeripheralSettings_fields,
                                                     api_PeripheralSettings_size,
                                                     PeripheralSettings_defaults()};

    Robot() { persistence.readFromFS(); }

    StateUpdateResult setPins(int sda, int scl) {
        PeripheralSettings wanted = PeripheralSettings_defaults();
        wanted.sda = sda;
        wanted.scl = scl;
        return service.update([&](PeripheralSettings &s) { return PeripheralSettings_update(wanted, s); }, "test");
    }

    PeripheralSettings settings() { return service.state(); }
};

std::vector<uint8_t> contents(const std::string &path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

bool cutsPower(const std::function<void()> &save) {
    try {
        save();
    } catch (const PowerCut &) {
        return true;
    }
    return false;
}

void write(const std::string &path, const std::vector<uint8_t> &bytes) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

} // namespace

void setUp() {
    std::filesystem::remove_all(DIRECTORY);
    nextMove = Move::MOVES;
    movesCut = 0;
}
void tearDown() {}

void test_saved_settings_survive_a_restart() {
    {
        Robot robot;
        TEST_ASSERT_EQUAL(StateUpdateResult::CHANGED, robot.setPins(5, 6));
    }
    Robot restarted;
    TEST_ASSERT_EQUAL(5, restarted.settings().sda);
    TEST_ASSERT_EQUAL(6, restarted.settings().scl);
    TEST_ASSERT_FALSE(std::filesystem::exists(BACKUP_PATH));
}

// The save has written the new settings in full and the board resets before they are moved over the stored ones.
void test_a_reset_before_the_move_keeps_the_stored_settings() {
    {
        Robot robot;
        robot.setPins(5, 6);
        nextMove = Move::CUTS_POWER;
        TEST_ASSERT_TRUE(cutsPower([&] { robot.setPins(7, 8); }));
    }
    TEST_ASSERT_EQUAL(1, movesCut);
    nextMove = Move::MOVES;
    Robot restarted;
    TEST_ASSERT_EQUAL(5, restarted.settings().sda);
    TEST_ASSERT_EQUAL(6, restarted.settings().scl);
    TEST_ASSERT_FALSE(std::filesystem::exists(BACKUP_PATH));
}

// A reset in the middle of writing leaves part of a save beside the stored file: the next boot loads the stored
// settings, and the next save replaces the leftover.
void test_a_partial_save_left_by_a_reset_is_ignored_and_replaced() {
    {
        Robot robot;
        robot.setPins(5, 6);
    }
    write(TEMPORARY_PATH, {0x08});
    Robot restarted;
    TEST_ASSERT_EQUAL(5, restarted.settings().sda);
    TEST_ASSERT_EQUAL(6, restarted.settings().scl);
    restarted.setPins(7, 8);
    TEST_ASSERT_FALSE(std::filesystem::exists(TEMPORARY_PATH));
    Robot again;
    TEST_ASSERT_EQUAL(7, again.settings().sda);
    TEST_ASSERT_EQUAL(8, again.settings().scl);
}

// A move the file system refuses, as a full one does, fails the save and leaves neither the stored file removed
// nor the new one lying beside it.
void test_a_refused_move_keeps_the_stored_settings() {
    Robot robot;
    robot.setPins(5, 6);
    const std::vector<uint8_t> stored = contents(FILE_PATH);
    nextMove = Move::FAILS;
    robot.setPins(7, 8);
    TEST_ASSERT_FALSE(robot.persistence.writeToFS());
    TEST_ASSERT_TRUE(contents(FILE_PATH) == stored);
    TEST_ASSERT_FALSE(std::filesystem::exists(TEMPORARY_PATH));
    nextMove = Move::MOVES;
    Robot restarted;
    TEST_ASSERT_EQUAL(5, restarted.settings().sda);
    TEST_ASSERT_EQUAL(6, restarted.settings().scl);
}

// A file that does not load replaces the backup an earlier one left, and the defaults are stored in its place.
void test_an_unloadable_file_is_kept_over_an_older_backup() {
    std::filesystem::create_directories(DIRECTORY + "/config");
    write(BACKUP_PATH, {0x01, 0x02});
    const std::vector<uint8_t> unloadable = {0xFF, 0xFF, 0xFF};
    write(FILE_PATH, unloadable);
    {
        Robot robot;
        TEST_ASSERT_EQUAL(SDA_PIN, robot.settings().sda);
    }
    TEST_ASSERT_TRUE(contents(BACKUP_PATH) == unloadable);
    Robot restarted;
    TEST_ASSERT_EQUAL(SDA_PIN, restarted.settings().sda);
    TEST_ASSERT_EQUAL(SCL_PIN, restarted.settings().scl);
    TEST_ASSERT_TRUE(contents(BACKUP_PATH) == unloadable);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_saved_settings_survive_a_restart);
    RUN_TEST(test_a_reset_before_the_move_keeps_the_stored_settings);
    RUN_TEST(test_a_partial_save_left_by_a_reset_is_ignored_and_replaced);
    RUN_TEST(test_a_refused_move_keeps_the_stored_settings);
    RUN_TEST(test_an_unloadable_file_is_kept_over_an_older_backup);
    return UNITY_END();
}
