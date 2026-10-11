// Host tests for the chunked file writer the app uses over BLE and serial, where there is no HTTP.

#include <unity.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <file_transfer.h>

namespace {

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

}  // namespace

void setUp() {}
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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_paths_must_be_absolute_inside_the_mount);
    RUN_TEST(test_the_settings_directory_is_out_of_reach);
    RUN_TEST(test_chunks_assemble_the_file_in_order);
    RUN_TEST(test_a_complete_upload_replaces_a_longer_previous_file);
    RUN_TEST(test_an_unfinished_upload_leaves_the_previous_file_intact);
    RUN_TEST(test_a_chunk_past_the_total_or_past_the_end_is_refused);
    RUN_TEST(test_a_refused_chunk_touches_no_file);
    RUN_TEST(test_read_returns_the_window_and_the_total);
    return UNITY_END();
}
