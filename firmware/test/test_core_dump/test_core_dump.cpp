// Host tests of the core-dump readout: the summary the panic handler left, as the status page is sent it; the dump's
// bytes, chunk by chunk, as CoreDumpRead and GET /api/coredump serve them; and whether the running build's ELF
// decodes the dump. The coredump partition is a buffer here; core_dump_flash.h reads the real one.

#include <unity.h>

#include <cstring>
#include <string>
#include <vector>
#include <core_dump.h>

namespace {

const std::string RUNNING = "3f9a1c0be47d5a2961c8f0e3b7a4d6c2e9f1a8b5c3d7e0f2a4b6c8d0e1f3a5b7";

// The header's second word as an ESP32-S3 (chip ID 9) writes it for an ELF dump with a CRC32 checksum.
constexpr uint32_t ESP32S3_ELF_CRC32 = 0x00090102;

// A dump in a buffer, with the summary the IDF would read from it. Every call is counted: each is a flash access on
// the robot, and a summary checks the whole dump first.
class FakeStore : public core_dump::Store {
  public:
    std::vector<uint8_t> image;
    core_dump::Summary crash {};
    bool checksumHolds = true;
    bool readFails = false;
    bool eraseFails = false;
    int sizes = 0, summaries = 0, reads = 0;

    uint32_t size() override {
        sizes++;
        return image.size();
    }

    bool summary(core_dump::Summary& out) override {
        summaries++;
        if (!checksumHolds) return false;
        out = crash;
        return true;
    }

    bool read(uint32_t offset, uint8_t* into, size_t length) override {
        reads++;
        if (readFails || offset + length > image.size()) return false;
        memcpy(into, image.data() + offset, length);
        return true;
    }

    bool erase() override {
        if (eraseFails) return false;
        image.clear();
        return true;
    }

    void writeVersion(uint32_t version) { memcpy(image.data() + 4, &version, sizeof(version)); }

    int accesses() const { return sizes + summaries + reads; }
};

core_dump::Summary watchdogCrash() {
    core_dump::Summary crash {};
    strcpy(crash.task, "control");
    crash.pc = 0x42012345;
    crash.depth = 3;
    crash.backtrace[0] = 0x42012345;
    crash.backtrace[1] = 0x42023456;
    crash.backtrace[2] = 0x40378abc;
    strcpy(crash.panicReason, "Task watchdog got triggered.");
    strcpy(crash.elfSha256, RUNNING.c_str());
    return crash;
}

FakeStore storeWith(size_t bytes) {
    FakeStore store;
    for (size_t i = 0; i < bytes; i++) store.image.push_back(static_cast<uint8_t>(i * 7));
    store.writeVersion(ESP32S3_ELF_CRC32);
    store.crash = watchdogCrash();
    return store;
}

socket_message_CoreDumpReport reportOf(core_dump::Store& store, const std::string& running = RUNNING) {
    socket_message_CoreDumpReport report;
    memset(&report, 0x5a, sizeof(report)); // whatever the adapter's stack held
    core_dump::report(store, running.c_str(), report);
    return report;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_no_dump_reports_nothing_stored_and_no_crash() {
    FakeStore store;
    const auto report = reportOf(store);
    TEST_ASSERT_FALSE(report.stored);
    TEST_ASSERT_EQUAL_UINT32(0, report.size);
    TEST_ASSERT_FALSE(report.readable);
    TEST_ASSERT_EQUAL_STRING("", report.task);
    TEST_ASSERT_EQUAL_UINT32(0, report.backtrace_count);
    TEST_ASSERT_FALSE(report.elf_matches_running);
}

void test_a_stored_dump_reports_its_crash() {
    FakeStore store = storeWith(40000);
    const auto report = reportOf(store);
    TEST_ASSERT_TRUE(report.stored);
    TEST_ASSERT_EQUAL_UINT32(40000, report.size);
    TEST_ASSERT_TRUE(report.readable);
    TEST_ASSERT_EQUAL_STRING("control", report.task);
    TEST_ASSERT_EQUAL_HEX32(0x42012345, report.pc);
    TEST_ASSERT_EQUAL_UINT32(3, report.backtrace_count);
    const uint32_t expected[] = {0x42012345, 0x42023456, 0x40378abc};
    TEST_ASSERT_EQUAL_HEX32_ARRAY(expected, report.backtrace, 3);
    TEST_ASSERT_FALSE(report.backtrace_corrupted);
    TEST_ASSERT_EQUAL_STRING("Task watchdog got triggered.", report.panic_reason);
    TEST_ASSERT_EQUAL_STRING(RUNNING.c_str(), report.app_elf_sha256);
    TEST_ASSERT_TRUE(report.elf_matches_running);
}

// A dump cut short by a reset fails its checksum: its size is known, and nothing read from it is shown.
void test_a_dump_that_fails_its_checksum_is_stored_but_unread() {
    FakeStore store = storeWith(4096);
    store.checksumHolds = false;
    const auto report = reportOf(store);
    TEST_ASSERT_TRUE(report.stored);
    TEST_ASSERT_EQUAL_UINT32(4096, report.size);
    TEST_ASSERT_FALSE(report.readable);
    TEST_ASSERT_EQUAL_STRING("", report.task);
    TEST_ASSERT_EQUAL_UINT32(0, report.pc);
    TEST_ASSERT_EQUAL_UINT32(0, report.backtrace_count);
    TEST_ASSERT_EQUAL_STRING("", report.app_elf_sha256);
    TEST_ASSERT_FALSE(report.elf_matches_running);
}

// A depth past the reply's room, as the IDF itself reports for a deep stack, must not write past the reply.
void test_the_backtrace_is_cut_to_the_replys_room() {
    FakeStore store = storeWith(1024);
    store.crash.depth = 1000;
    for (uint32_t i = 0; i < 16; i++) store.crash.backtrace[i] = 0x42000000 + i;
    store.crash.backtraceCorrupted = true;
    const auto report = reportOf(store);
    TEST_ASSERT_EQUAL_UINT32(16, report.backtrace_count);
    TEST_ASSERT_EQUAL_HEX32(0x4200000f, report.backtrace[15]);
    TEST_ASSERT_TRUE(report.backtrace_corrupted);
}

// The summary's strings are fixed arrays from flash: one that fills its array has no terminator of its own.
void test_unterminated_strings_from_flash_arrive_terminated() {
    FakeStore store = storeWith(1024);
    memset(store.crash.task, 'T', sizeof(store.crash.task));
    memset(store.crash.panicReason, 'R', sizeof(store.crash.panicReason));
    memset(store.crash.elfSha256, 'a', sizeof(store.crash.elfSha256));
    const auto report = reportOf(store);
    TEST_ASSERT_EQUAL_size_t(sizeof(report.task) - 1, strlen(report.task));
    TEST_ASSERT_EQUAL_size_t(sizeof(report.panic_reason) - 1, strlen(report.panic_reason));
    TEST_ASSERT_EQUAL_size_t(64, strlen(report.app_elf_sha256));
}

void test_a_dump_from_another_build_does_not_match_the_running_one() {
    FakeStore store = storeWith(1024);
    strcpy(store.crash.elfSha256, "0000000000000000000000000000000000000000000000000000000000000000");
    TEST_ASSERT_FALSE(reportOf(store).elf_matches_running);
}

// A firmware built before CONFIG_APP_RETRIEVE_LEN_ELF_SHA=64 kept the IDF's default of 9 digits.
void test_a_short_hash_matches_the_build_it_begins() {
    TEST_ASSERT_TRUE(core_dump::sameFirmware(RUNNING.substr(0, 9).c_str(), RUNNING.c_str()));
    TEST_ASSERT_FALSE(core_dump::sameFirmware("3f9a1c0bf", RUNNING.c_str()));
}

// Fewer digits than the IDF can keep (8) name no build: an empty hash would begin every one.
void test_a_hash_too_short_to_name_a_build_matches_none() {
    TEST_ASSERT_FALSE(core_dump::sameFirmware("", RUNNING.c_str()));
    TEST_ASSERT_FALSE(core_dump::sameFirmware(RUNNING.substr(0, 7).c_str(), RUNNING.c_str()));
    TEST_ASSERT_TRUE(core_dump::sameFirmware(RUNNING.substr(0, 8).c_str(), RUNNING.c_str()));
}

void test_a_hash_longer_than_the_running_one_does_not_match() {
    TEST_ASSERT_FALSE(core_dump::sameFirmware((RUNNING + "0").c_str(), RUNNING.c_str()));
}

void test_chunks_read_back_the_whole_dump() {
    FakeStore store = storeWith(1300);
    std::vector<uint8_t> read;
    uint32_t total = 0;
    do {
        api_FileChunk chunk {};
        TEST_ASSERT_EQUAL_UINT32(200, core_dump::readChunk(store, read.size(), 512, chunk));
        TEST_ASSERT_TRUE(chunk.content.size > 0);
        read.insert(read.end(), chunk.content.bytes, chunk.content.bytes + chunk.content.size);
        total = chunk.total_size;
    } while (read.size() < total);
    TEST_ASSERT_EQUAL_UINT32(1300, total);
    TEST_ASSERT_TRUE(read == store.image);
}

void test_a_read_past_the_end_returns_no_bytes() {
    FakeStore store = storeWith(1300);
    api_FileChunk chunk {};
    TEST_ASSERT_EQUAL_UINT32(200, core_dump::readChunk(store, 1300, 512, chunk));
    TEST_ASSERT_EQUAL_UINT32(0, chunk.content.size);
    TEST_ASSERT_EQUAL_UINT32(1300, chunk.total_size);
    TEST_ASSERT_EQUAL_UINT32(200, core_dump::readChunk(store, 0xfffffff0, 512, chunk));
    TEST_ASSERT_EQUAL_UINT32(0, chunk.content.size);
}

void test_a_chunk_larger_than_the_reply_holds_is_refused() {
    FakeStore store = storeWith(1300);
    api_FileChunk chunk {};
    TEST_ASSERT_EQUAL_UINT32(400, core_dump::readChunk(store, 0, sizeof(chunk.content.bytes) + 1, chunk));
    TEST_ASSERT_EQUAL_UINT32(0, chunk.content.size);
}

void test_reading_without_a_dump_is_not_found() {
    FakeStore store;
    api_FileChunk chunk {};
    TEST_ASSERT_EQUAL_UINT32(404, core_dump::readChunk(store, 0, 512, chunk));
    TEST_ASSERT_EQUAL_UINT32(0, chunk.content.size);
}

void test_a_flash_read_that_fails_is_a_server_error() {
    FakeStore store = storeWith(1300);
    store.readFails = true;
    api_FileChunk chunk {};
    TEST_ASSERT_EQUAL_UINT32(500, core_dump::readChunk(store, 0, 512, chunk));
    TEST_ASSERT_EQUAL_UINT32(0, chunk.content.size);
}

// A BIN-format dump, or an ELF one with a SHA-256 checksum, is not what this build's summary parses: its offsets would
// be read as an ELF's, wherever they point.
void test_a_dump_in_another_format_is_stored_but_unread() {
    for (const uint32_t version : {0x00090003u, 0x00090001u, 0x00090103u}) {
        FakeStore store = storeWith(4096);
        store.writeVersion(version);
        const auto report = reportOf(store);
        TEST_ASSERT_TRUE(report.stored);
        TEST_ASSERT_EQUAL_UINT32(4096, report.size);
        TEST_ASSERT_FALSE(report.readable);
        TEST_ASSERT_EQUAL_STRING("", report.task);
        TEST_ASSERT_EQUAL_INT(0, store.summaries);
    }
}

void test_a_dump_whose_header_cannot_be_read_is_unread() {
    FakeStore store = storeWith(4096);
    store.readFails = true;
    const auto report = reportOf(store);
    TEST_ASSERT_TRUE(report.stored);
    TEST_ASSERT_FALSE(report.readable);
    TEST_ASSERT_EQUAL_INT(0, store.summaries);
}

// Checking a dump reads all of it from flash, parking the control task's core for each read: it runs once at boot,
// and the status page's questions are answered from it.
void test_the_dump_is_checked_once_and_every_report_answered_from_that() {
    FakeStore store = storeWith(1300);
    core_dump::Readout readout(store);
    readout.check(RUNNING.c_str());
    TEST_ASSERT_EQUAL_INT(1, store.summaries);
    store.sizes = store.summaries = store.reads = 0;
    for (int i = 0; i < 2; i++) {
        socket_message_CoreDumpReport report;
        memset(&report, 0x5a, sizeof(report));
        readout.report(report);
        TEST_ASSERT_TRUE(report.readable);
        TEST_ASSERT_EQUAL_STRING("control", report.task);
        TEST_ASSERT_TRUE(report.elf_matches_running);
    }
    TEST_ASSERT_EQUAL_INT(0, store.accesses());
}

// Erasing flash stalls both cores, so only a robot at rest has its dump erased; a refused erase answers with the
// report from the check, touching no flash while the robot moves.
void test_an_erase_is_refused_while_the_robot_moves_without_a_check() {
    FakeStore store = storeWith(1300);
    core_dump::Readout readout(store);
    readout.check(RUNNING.c_str());
    store.sizes = store.summaries = store.reads = 0;
    socket_message_CoreDumpReport report {};
    TEST_ASSERT_EQUAL_UINT32(409, readout.erase(false, RUNNING.c_str(), report));
    TEST_ASSERT_TRUE(report.stored);
    TEST_ASSERT_EQUAL_STRING("control", report.task);
    TEST_ASSERT_EQUAL_INT(0, store.accesses());
    TEST_ASSERT_EQUAL_size_t(1300, store.image.size());
}

void test_an_erase_at_rest_leaves_no_dump_in_any_later_report() {
    FakeStore store = storeWith(1300);
    core_dump::Readout readout(store);
    readout.check(RUNNING.c_str());
    socket_message_CoreDumpReport report {};
    TEST_ASSERT_EQUAL_UINT32(200, readout.erase(true, RUNNING.c_str(), report));
    TEST_ASSERT_FALSE(report.stored);
    TEST_ASSERT_EQUAL_size_t(0, store.image.size());
    socket_message_CoreDumpReport later;
    memset(&later, 0x5a, sizeof(later));
    readout.report(later);
    TEST_ASSERT_FALSE(later.stored);
    TEST_ASSERT_FALSE(later.readable);
    TEST_ASSERT_EQUAL_STRING("", later.task);
}

// After a failed erase the partition holds whatever the erase left, so the dump is checked again, at rest.
void test_an_erase_the_flash_refuses_checks_the_dump_again() {
    FakeStore store = storeWith(1300);
    store.eraseFails = true;
    core_dump::Readout readout(store);
    readout.check(RUNNING.c_str());
    store.crash.pc = 0x42054321;
    socket_message_CoreDumpReport report {};
    TEST_ASSERT_EQUAL_UINT32(500, readout.erase(true, RUNNING.c_str(), report));
    TEST_ASSERT_EQUAL_INT(2, store.summaries);
    TEST_ASSERT_TRUE(report.stored);
    TEST_ASSERT_EQUAL_HEX32(0x42054321, report.pc);
    socket_message_CoreDumpReport later {};
    readout.report(later);
    TEST_ASSERT_EQUAL_HEX32(0x42054321, later.pc);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_no_dump_reports_nothing_stored_and_no_crash);
    RUN_TEST(test_a_stored_dump_reports_its_crash);
    RUN_TEST(test_a_dump_that_fails_its_checksum_is_stored_but_unread);
    RUN_TEST(test_the_backtrace_is_cut_to_the_replys_room);
    RUN_TEST(test_unterminated_strings_from_flash_arrive_terminated);
    RUN_TEST(test_a_dump_from_another_build_does_not_match_the_running_one);
    RUN_TEST(test_a_short_hash_matches_the_build_it_begins);
    RUN_TEST(test_a_hash_too_short_to_name_a_build_matches_none);
    RUN_TEST(test_a_hash_longer_than_the_running_one_does_not_match);
    RUN_TEST(test_chunks_read_back_the_whole_dump);
    RUN_TEST(test_a_read_past_the_end_returns_no_bytes);
    RUN_TEST(test_a_chunk_larger_than_the_reply_holds_is_refused);
    RUN_TEST(test_reading_without_a_dump_is_not_found);
    RUN_TEST(test_a_flash_read_that_fails_is_a_server_error);
    RUN_TEST(test_a_dump_in_another_format_is_stored_but_unread);
    RUN_TEST(test_a_dump_whose_header_cannot_be_read_is_unread);
    RUN_TEST(test_the_dump_is_checked_once_and_every_report_answered_from_that);
    RUN_TEST(test_an_erase_is_refused_while_the_robot_moves_without_a_check);
    RUN_TEST(test_an_erase_at_rest_leaves_no_dump_in_any_later_report);
    RUN_TEST(test_an_erase_the_flash_refuses_checks_the_dump_again);
    return UNITY_END();
}
