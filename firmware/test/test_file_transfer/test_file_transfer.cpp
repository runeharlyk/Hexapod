// Host tests for the file transfer the app uses over BLE, serial and WebSocket: chunked read and write, listing, mkdir.

#include <unity.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>
#include <dirent.h>
// The Windows C runtime's dirent has no d_type and its headers name no DT_ values; these are newlib's (the ESP-IDF's
// sys/dirent.h), so the header's d_type branch builds on that host too and is tested on an entry that carries one.
#ifndef DT_UNKNOWN
#define DT_UNKNOWN 0
#define DT_REG 1
#define DT_DIR 2
#endif
#include <file_transfer.h>
#include <filesystem.h>
#include <platform_shared/api.pb.h>

namespace {

// Whether the file system moves a finished upload over its destination, as a full one does not.
bool moveRefused = false;

std::string tmpFile() {
    static int n = 0;
    return ".pio/test_file_transfer_" + std::to_string(n++) + ".bin";
}

std::vector<uint8_t> contents(const std::string &path) {
    std::vector<uint8_t> out;
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return out;
    uint8_t buf[64];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) out.insert(out.end(), buf, buf + n);
    fclose(f);
    return out;
}

// A directory tree standing in for the mounted file system, emptied before every test.
const char *const ROOT = ".pio/test_file_transfer_root";

void touch(const std::string &rel, size_t bytes = 0) {
    const std::filesystem::path path = std::string(ROOT) + rel;
    std::filesystem::create_directories(path.parent_path());
    FILE *f = fopen(path.string().c_str(), "wb");
    const std::vector<uint8_t> data(bytes, 0xAB);
    if (bytes) fwrite(data.data(), 1, bytes, f);
    fclose(f);
}

void makeDirectory(const std::string &rel) { std::filesystem::create_directories(std::string(ROOT) + rel); }

bool exists(const std::string &rel) { return std::filesystem::exists(std::string(ROOT) + rel); }

bool isDirectory(const std::string &rel) { return std::filesystem::is_directory(std::string(ROOT) + rel); }

std::vector<std::string> names(const api_FileList &list) {
    std::vector<std::string> out;
    for (pb_size_t i = 0; i < list.entries_count; ++i) out.push_back(list.entries[i].name);
    return out;
}

constexpr size_t PAGE = sizeof(api_FileList::entries) / sizeof(api_FileEntry);

}  // namespace

namespace FileSystem {

bool mkdirRecursive(const char *path) {
    std::error_code error;
    std::filesystem::create_directories(path, error);
    return !error;
}

// Moves over an existing destination, as LittleFS does; the Windows C runtime's rename refuses to.
bool replaceFile(const char *from, const char *to) {
    if (moveRefused) return false;
    std::error_code error;
    std::filesystem::rename(from, to, error);
    return !error;
}

}  // namespace FileSystem

void setUp() {
    moveRefused = false;
    std::filesystem::remove_all(ROOT);
    std::filesystem::create_directories(ROOT);
}
void tearDown() {}

void test_paths_must_be_absolute_inside_the_mount() {
    TEST_ASSERT_TRUE(file_transfer::validPath("/animations/wave.pb"));
    TEST_ASSERT_FALSE(file_transfer::validPath("animations/wave.pb"));
    TEST_ASSERT_FALSE(file_transfer::validPath("/animations/../config/x"));
    TEST_ASSERT_FALSE(file_transfer::validPath(""));
    TEST_ASSERT_FALSE(file_transfer::validPath(nullptr));
}

void test_the_settings_directory_is_out_of_reach() {
    TEST_ASSERT_FALSE(file_transfer::validPath("/config"));
    TEST_ASSERT_FALSE(file_transfer::validPath("/config/"));
    TEST_ASSERT_FALSE(file_transfer::validPath("/config/wifiSettings.json"));
    TEST_ASSERT_FALSE(file_transfer::validPath("//config/wifiSettings.json"));
    TEST_ASSERT_FALSE(file_transfer::validPath("/./config/apSettings.json"));
    TEST_ASSERT_TRUE(file_transfer::validPath("/"));
    TEST_ASSERT_TRUE(file_transfer::validPath("/configs/x"));
    TEST_ASSERT_TRUE(file_transfer::validPath("/config.json"));
    TEST_ASSERT_TRUE(file_transfer::validPath("/animations/config"));
}

void test_chunks_assemble_the_file_in_order() {
    const std::string path = tmpFile();
    const uint8_t a[3] = {1, 2, 3}, b[2] = {4, 5};
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 0, 5, a, 3));
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 3, 5, b, 2));
    const std::vector<uint8_t> got = contents(path);
    TEST_ASSERT_EQUAL(5, got.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a, got.data(), 3);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(b, got.data() + 3, 2);
    remove(path.c_str());
}

void test_a_complete_upload_replaces_a_longer_previous_file() {
    const std::string path = tmpFile();
    const uint8_t a[4] = {9, 9, 9, 9}, b[1] = {1};
    file_transfer::write(path.c_str(), 0, 4, a, 4);
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 0, 1, b, 1));
    const std::vector<uint8_t> got = contents(path);
    TEST_ASSERT_EQUAL(1, got.size());
    TEST_ASSERT_EQUAL_UINT8(1, got[0]);
    TEST_ASSERT_EQUAL(-1, file_transfer::fileSize(file_transfer::partPath(path.c_str()).c_str()));
    remove(path.c_str());
}

void test_an_unfinished_upload_leaves_the_previous_file_intact() {
    const std::string path = tmpFile();
    const uint8_t old[3] = {5, 6, 7}, fresh[2] = {1, 2};
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 0, 3, old, 3));
    // The new upload declares four bytes and stops after two, as a dropped link would.
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 0, 4, fresh, 2));
    const std::vector<uint8_t> got = contents(path);
    TEST_ASSERT_EQUAL(3, got.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(old, got.data(), 3);
    remove(path.c_str());
    remove(file_transfer::partPath(path.c_str()).c_str());
}

// The last chunk is written in full and the file system then refuses to move it into place: the upload fails, and
// the previous file stays as it was rather than being removed before a move that cannot succeed.
void test_a_refused_move_keeps_the_previous_file() {
    const std::string path = tmpFile();
    const uint8_t old[3] = {5, 6, 7}, fresh[2] = {1, 2};
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 0, 3, old, 3));
    moveRefused = true;
    TEST_ASSERT_EQUAL(500, file_transfer::write(path.c_str(), 0, 2, fresh, 2));
    const std::vector<uint8_t> got = contents(path);
    TEST_ASSERT_EQUAL(3, got.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(old, got.data(), 3);
    remove(path.c_str());
    remove(file_transfer::partPath(path.c_str()).c_str());
}

void test_a_chunk_past_the_total_or_past_the_end_is_refused() {
    const std::string path = tmpFile();
    const uint8_t a[3] = {1, 2, 3};
    TEST_ASSERT_EQUAL(400, file_transfer::write(path.c_str(), 0, 2, a, 3));
    TEST_ASSERT_EQUAL(0, contents(path).size());
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 0, 6, a, 3));
    TEST_ASSERT_EQUAL(400, file_transfer::write(path.c_str(), 4, 6, a, 2));  // gap after byte 3
    const std::string part = file_transfer::partPath(path.c_str());
    TEST_ASSERT_EQUAL(3, contents(part).size());
    static uint8_t big[file_transfer::CHUNK_MAX + 1];
    TEST_ASSERT_EQUAL(400, file_transfer::write(path.c_str(), 3, 1000, big, sizeof(big)));
    remove(part.c_str());
}

void test_a_refused_chunk_touches_no_file() {
    const std::string path = tmpFile();
    const uint8_t a[4] = {7, 7, 7, 7};
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 0, 4, a, 4));
    // An overshooting first chunk must be refused before offset 0 truncates the file.
    TEST_ASSERT_EQUAL(400, file_transfer::write(path.c_str(), 0, 3, a, 4));
    const std::vector<uint8_t> got = contents(path);
    TEST_ASSERT_EQUAL(4, got.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a, got.data(), 4);
    remove(path.c_str());

    const std::string missing = tmpFile();
    TEST_ASSERT_EQUAL(400, file_transfer::write(missing.c_str(), 2, 6, a, 2));
    TEST_ASSERT_EQUAL(-1, file_transfer::fileSize(missing.c_str()));
}

void test_read_returns_the_window_and_the_total() {
    const std::string path = tmpFile();
    uint8_t data[700];
    for (int i = 0; i < 700; ++i) data[i] = (uint8_t)i;
    file_transfer::write(path.c_str(), 0, 700, data, 512);
    file_transfer::write(path.c_str(), 512, 700, data + 512, 188);
    uint8_t out[512];
    size_t n = 0;
    uint32_t total = 0;
    TEST_ASSERT_EQUAL(200, file_transfer::read(path.c_str(), 512, 512, out, n, total));
    TEST_ASSERT_EQUAL(188, n);
    TEST_ASSERT_EQUAL_UINT32(700, total);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data + 512, out, 188);
    TEST_ASSERT_EQUAL(404, file_transfer::read("/nonexistent/x.bin", 0, 10, out, n, total));
    TEST_ASSERT_EQUAL(400, file_transfer::read(path.c_str(), 0, 513, out, n, total));
    remove(path.c_str());
}

void test_a_listing_is_sorted_directories_first() {
    touch("/b.txt", 7);
    touch("/A.txt");
    makeDirectory("/zoo");
    makeDirectory("/animations");
    api_FileList list = api_FileList_init_zero;
    TEST_ASSERT_EQUAL(200, file_transfer::list("/", 0, list, ROOT));
    // Byte order within each group: "A.txt" sorts before "b.txt".
    const std::vector<std::string> expected = {"animations", "zoo", "A.txt", "b.txt"};
    TEST_ASSERT_TRUE(expected == names(list));
    TEST_ASSERT_EQUAL_UINT32(4, list.total);
    TEST_ASSERT_TRUE(list.entries[0].is_directory);
    TEST_ASSERT_EQUAL_UINT32(0, list.entries[0].size);
    TEST_ASSERT_FALSE(list.entries[3].is_directory);
    TEST_ASSERT_EQUAL_UINT32(7, list.entries[3].size);
}

void test_a_subdirectory_lists_only_its_own_entries() {
    touch("/animations/wave.pb", 12);
    touch("/top.txt");
    api_FileList list = api_FileList_init_zero;
    TEST_ASSERT_EQUAL(200, file_transfer::list("/animations", 0, list, ROOT));
    TEST_ASSERT_EQUAL(1, list.entries_count);
    TEST_ASSERT_EQUAL_STRING("wave.pb", list.entries[0].name);
    TEST_ASSERT_EQUAL_UINT32(12, list.entries[0].size);
    TEST_ASSERT_EQUAL_UINT32(1, list.total);
}

void test_a_large_directory_is_listed_in_pages_each_entry_once() {
    const size_t count = 2 * PAGE + 5;
    for (size_t i = 0; i < count; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "/f%03u", (unsigned)i);
        touch(name);
    }
    std::vector<std::string> seen;
    for (uint32_t offset = 0; offset < count; offset += PAGE) {
        api_FileList list = api_FileList_init_zero;
        TEST_ASSERT_EQUAL(200, file_transfer::list("/", offset, list, ROOT));
        TEST_ASSERT_EQUAL_UINT32(count, list.total);
        TEST_ASSERT_EQUAL(std::min(PAGE, count - offset), list.entries_count);
        for (const std::string &name : names(list)) seen.push_back(name);
    }
    TEST_ASSERT_EQUAL(count, seen.size());
    TEST_ASSERT_TRUE(std::is_sorted(seen.begin(), seen.end()));
    TEST_ASSERT_TRUE(std::adjacent_find(seen.begin(), seen.end()) == seen.end());
}

// Refactor guard (the first implementation was correct, only unbounded in memory, and the next read every entry's size
// on every scan): every window of a directory many pages long, at offsets on and off the page boundaries, equals the
// same window of the independently sorted whole, sizes included.
void test_every_page_of_a_mixed_directory_matches_the_sorted_listing_at_any_offset() {
    const size_t count = 4 * PAGE + 11;
    struct Expected {
        std::string name;
        bool directory;
        uint32_t size;
    };
    std::vector<Expected> expected;  // in listing order once sorted
    for (size_t i = 0; i < count; ++i) {
        const size_t shuffled = (i * 37) % count;  // 37 is coprime with 115: a permutation, not creation order
        const bool directory = shuffled % 3 == 0;
        char name[16];
        snprintf(name, sizeof(name), "%c%03u", shuffled % 2 ? 'a' : 'B', (unsigned)shuffled);
        if (directory) makeDirectory(std::string("/") + name);
        else touch(std::string("/") + name, shuffled);
        expected.push_back({name, directory, directory ? 0u : (uint32_t)shuffled});
    }
    std::sort(expected.begin(), expected.end(), [](const Expected &a, const Expected &b) {
        return a.directory != b.directory ? a.directory : a.name < b.name;
    });
    for (uint32_t offset = 0; offset <= count + PAGE; offset += 7) {
        api_FileList list = api_FileList_init_zero;
        TEST_ASSERT_EQUAL(200, file_transfer::list("/", offset, list, ROOT));
        TEST_ASSERT_EQUAL_UINT32(count, list.total);
        const size_t first = std::min<size_t>(offset, count);
        TEST_ASSERT_EQUAL(std::min(PAGE, count - first), list.entries_count);
        for (size_t i = 0; i < list.entries_count; ++i) {
            TEST_ASSERT_EQUAL_STRING(expected[first + i].name.c_str(), list.entries[i].name);
            TEST_ASSERT_EQUAL(expected[first + i].directory, list.entries[i].is_directory);
            TEST_ASSERT_EQUAL_UINT32(expected[first + i].size, list.entries[i].size);
        }
    }
}

// esp_littlefs fills d_type, which spares a stat() per entry of every scan; a dirent without it (the Windows C
// runtime's, which the listings above run on) or one left DT_UNKNOWN is asked of stat() instead.
struct TypedEntry {
    unsigned char d_type;
};

void test_an_entry_is_a_directory_by_its_d_type_or_else_by_stat() {
    makeDirectory("/dir");
    touch("/file");
    const std::string dir = std::string(ROOT) + "/dir", file = std::string(ROOT) + "/file",
                      missing = std::string(ROOT) + "/missing";
    // A d_type is taken as it is, without a stat(), which would contradict it here or find nothing.
    TEST_ASSERT_TRUE(file_transfer::detail::isDirectory(TypedEntry{DT_DIR}, missing.c_str()));
    TEST_ASSERT_FALSE(file_transfer::detail::isDirectory(TypedEntry{DT_REG}, dir.c_str()));
    TEST_ASSERT_TRUE(file_transfer::detail::isDirectory(TypedEntry{DT_UNKNOWN}, dir.c_str()));
    TEST_ASSERT_FALSE(file_transfer::detail::isDirectory(TypedEntry{DT_UNKNOWN}, file.c_str()));
    const dirent untyped{};
    TEST_ASSERT_TRUE(file_transfer::detail::isDirectory(untyped, dir.c_str()));
    TEST_ASSERT_FALSE(file_transfer::detail::isDirectory(untyped, file.c_str()));
}

void test_an_offset_past_the_end_is_an_empty_page_with_the_total() {
    touch("/a");
    touch("/b");
    api_FileList list = api_FileList_init_zero;
    TEST_ASSERT_EQUAL(200, file_transfer::list("/", 2, list, ROOT));
    TEST_ASSERT_EQUAL(0, list.entries_count);
    TEST_ASSERT_EQUAL_UINT32(2, list.total);
    TEST_ASSERT_EQUAL(200, file_transfer::list("/", 1000, list, ROOT));
    TEST_ASSERT_EQUAL(0, list.entries_count);
}

void test_a_missing_directory_is_404() {
    touch("/file.txt");
    api_FileList list = api_FileList_init_zero;
    TEST_ASSERT_EQUAL(404, file_transfer::list("/nowhere", 0, list, ROOT));
    TEST_ASSERT_EQUAL(404, file_transfer::list("/file.txt", 0, list, ROOT));
    TEST_ASSERT_EQUAL(0, list.entries_count);
}

void test_the_config_directory_is_refused() {
    touch("/config/wifiSettings.json", 9);
    api_FileList list = api_FileList_init_zero;
    TEST_ASSERT_EQUAL(400, file_transfer::list("/config", 0, list, ROOT));
    TEST_ASSERT_EQUAL(400, file_transfer::list("/config/", 0, list, ROOT));
    TEST_ASSERT_EQUAL(400, file_transfer::list("/animations/../config", 0, list, ROOT));
    TEST_ASSERT_EQUAL(400, file_transfer::list("/..", 0, list, ROOT));
    TEST_ASSERT_EQUAL(400, file_transfer::list("relative", 0, list, ROOT));
    TEST_ASSERT_EQUAL(400, file_transfer::list(nullptr, 0, list, ROOT));
    TEST_ASSERT_EQUAL(400, file_transfer::mkdir("/config", ROOT));
    TEST_ASSERT_EQUAL(400, file_transfer::mkdir("/config/sub", ROOT));
    TEST_ASSERT_EQUAL(400, file_transfer::mkdir("/x/../config/sub", ROOT));
    TEST_ASSERT_FALSE(exists("/config/sub"));
    TEST_ASSERT_FALSE(exists("/x"));
}

void test_the_root_listing_hides_config_but_not_lookalikes() {
    makeDirectory("/config");
    makeDirectory("/configs");
    touch("/config.json");
    makeDirectory("/animations/config");
    api_FileList list = api_FileList_init_zero;
    TEST_ASSERT_EQUAL(200, file_transfer::list("/", 0, list, ROOT));
    const std::vector<std::string> expected = {"animations", "configs", "config.json"};
    TEST_ASSERT_TRUE(expected == names(list));
    TEST_ASSERT_EQUAL_UINT32(3, list.total);
    TEST_ASSERT_EQUAL(200, file_transfer::list("/animations", 0, list, ROOT));
    TEST_ASSERT_EQUAL(1, list.entries_count);
}

void test_an_upload_in_progress_is_listed_as_its_part_file() {
    const std::string path = std::string(ROOT) + "/wave.pb";
    const uint8_t a[3] = {1, 2, 3};
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 0, 6, a, 3));
    api_FileList list = api_FileList_init_zero;
    TEST_ASSERT_EQUAL(200, file_transfer::list("/", 0, list, ROOT));
    TEST_ASSERT_EQUAL(1, list.entries_count);
    TEST_ASSERT_EQUAL_STRING("wave.pb.part", list.entries[0].name);
    TEST_ASSERT_EQUAL_UINT32(3, list.entries[0].size);
}

void test_a_name_longer_than_the_cap_is_skipped_and_not_counted() {
    const size_t cap = sizeof(api_FileEntry::name) - 1;
    const std::string fits(cap, 'a'), tooLong(cap + 1, 'b');
    touch("/" + fits);
    touch("/" + tooLong);
    touch("/short");
    api_FileList list = api_FileList_init_zero;
    TEST_ASSERT_EQUAL(200, file_transfer::list("/", 0, list, ROOT));
    const std::vector<std::string> expected = {fits, "short"};
    TEST_ASSERT_TRUE(expected == names(list));
    TEST_ASSERT_EQUAL_UINT32(2, list.total);
}

void test_mkdir_creates_and_accepts_an_existing_directory() {
    TEST_ASSERT_EQUAL(200, file_transfer::mkdir("/logs", ROOT));
    TEST_ASSERT_TRUE(isDirectory("/logs"));
    touch("/logs/a.txt");
    TEST_ASSERT_EQUAL(200, file_transfer::mkdir("/logs", ROOT));
    TEST_ASSERT_TRUE(exists("/logs/a.txt"));
    TEST_ASSERT_EQUAL(200, file_transfer::mkdir("/deep/er/still", ROOT));
    TEST_ASSERT_TRUE(isDirectory("/deep/er/still"));
    TEST_ASSERT_EQUAL(200, file_transfer::mkdir("/", ROOT));
}

void test_mkdir_over_a_file_is_409() {
    touch("/taken", 3);
    TEST_ASSERT_EQUAL(409, file_transfer::mkdir("/taken", ROOT));
    TEST_ASSERT_FALSE(isDirectory("/taken"));
    TEST_ASSERT_EQUAL(400, file_transfer::mkdir("taken", ROOT));
    TEST_ASSERT_EQUAL(400, file_transfer::mkdir(nullptr, ROOT));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_paths_must_be_absolute_inside_the_mount);
    RUN_TEST(test_the_settings_directory_is_out_of_reach);
    RUN_TEST(test_chunks_assemble_the_file_in_order);
    RUN_TEST(test_a_complete_upload_replaces_a_longer_previous_file);
    RUN_TEST(test_an_unfinished_upload_leaves_the_previous_file_intact);
    RUN_TEST(test_a_refused_move_keeps_the_previous_file);
    RUN_TEST(test_a_chunk_past_the_total_or_past_the_end_is_refused);
    RUN_TEST(test_a_refused_chunk_touches_no_file);
    RUN_TEST(test_read_returns_the_window_and_the_total);
    RUN_TEST(test_a_listing_is_sorted_directories_first);
    RUN_TEST(test_a_subdirectory_lists_only_its_own_entries);
    RUN_TEST(test_a_large_directory_is_listed_in_pages_each_entry_once);
    RUN_TEST(test_every_page_of_a_mixed_directory_matches_the_sorted_listing_at_any_offset);
    RUN_TEST(test_an_entry_is_a_directory_by_its_d_type_or_else_by_stat);
    RUN_TEST(test_an_offset_past_the_end_is_an_empty_page_with_the_total);
    RUN_TEST(test_a_missing_directory_is_404);
    RUN_TEST(test_the_config_directory_is_refused);
    RUN_TEST(test_the_root_listing_hides_config_but_not_lookalikes);
    RUN_TEST(test_an_upload_in_progress_is_listed_as_its_part_file);
    RUN_TEST(test_a_name_longer_than_the_cap_is_skipped_and_not_counted);
    RUN_TEST(test_mkdir_creates_and_accepts_an_existing_directory);
    RUN_TEST(test_mkdir_over_a_file_is_409);
    return UNITY_END();
}
