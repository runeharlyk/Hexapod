// Host tests of the serial framing against platform_shared/serial_frame_vectors.json, which the
// app's serial-framing.ts passes too, so the two ends cannot drift apart.

#include <unity.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <communication/serial_frame.h>

namespace {

using Bytes = std::vector<uint8_t>;

struct Vector {
    std::string name;
    Bytes stream;
    std::vector<Bytes> frames;
};

Bytes fromHex(const std::string& hex) {
    Bytes bytes;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) bytes.push_back(std::stoi(hex.substr(i, 2), nullptr, 16));
    return bytes;
}

std::vector<std::string> strings(const std::string& list) {
    std::vector<std::string> out;
    for (size_t open = list.find('"'); open != std::string::npos; open = list.find('"', open + 1)) {
        const size_t close = list.find('"', open + 1);
        out.push_back(list.substr(open + 1, close - open - 1));
        open = close;
    }
    return out;
}

// The text between `open` and the matching `close` that follows `key`, from `at` on; `at` moves past.
std::string after(const std::string& json, const std::string& key, char open, char close, size_t& at) {
    const size_t start = json.find(open, json.find(key, at) + key.size());
    const size_t end = json.find(close, start + 1);
    at = end + 1;
    return json.substr(start + 1, end - start - 1);
}

// The file lists each vector's keys in one order with no escapes in the values; this reads only that.
std::vector<Vector> readVectors() {
    std::ifstream file("platform_shared/serial_frame_vectors.json");
    std::stringstream text;
    text << file.rdbuf();
    const std::string json = text.str();
    std::vector<Vector> vectors;
    for (size_t at = json.find("\"vectors\""); json.find("\"name\":", at) != std::string::npos;) {
        Vector v;
        v.name = after(json, "\"name\":", '"', '"', at);
        v.stream = fromHex(after(json, "\"stream\":", '"', '"', at));
        for (const std::string& hex : strings(after(json, "\"frames\":", '[', ']', at))) v.frames.push_back(fromHex(hex));
        vectors.push_back(v);
    }
    return vectors;
}

std::vector<Bytes> decode(const Bytes& stream, size_t chunk) {
    std::vector<Bytes> frames;
    serial_frame::Decoder decoder([&](const uint8_t* m, size_t n) { frames.emplace_back(m, m + n); });
    for (size_t i = 0; i < stream.size(); i += chunk) decoder.feed(stream.data() + i, std::min(chunk, stream.size() - i));
    return frames;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_every_vector_decodes_alike_in_any_chunking() {
    const std::vector<Vector> vectors = readVectors();
    TEST_ASSERT_EQUAL_MESSAGE(7, vectors.size(), "vectors file missing or unreadable");
    for (const Vector& v : vectors) {
        for (size_t chunk : {v.stream.size(), size_t(1), size_t(7)}) {
            TEST_ASSERT_TRUE_MESSAGE(decode(v.stream, chunk) == v.frames, v.name.c_str());
        }
    }
}

void test_a_single_frame_encodes_to_its_vector() {
    int checked = 0;
    for (const Vector& v : readVectors()) {
        if (v.frames.size() != 1) continue;
        const Bytes encoded = serial_frame::encode(v.frames[0].data(), v.frames[0].size());
        // Vectors with text around the frame contain the encoding rather than equal it.
        TEST_ASSERT_TRUE_MESSAGE(std::search(v.stream.begin(), v.stream.end(), encoded.begin(), encoded.end()) !=
                                     v.stream.end(),
                                 v.name.c_str());
        checked++;
    }
    TEST_ASSERT_GREATER_THAN(3, checked);
}

void test_a_reader_starting_mid_frame_finds_the_next_frames() {
    const Bytes a = {8, 1, 0, 0, 7}, b = {0x10, 0x20};
    const Bytes first = serial_frame::encode(a.data(), a.size()), second = serial_frame::encode(b.data(), b.size());
    Bytes stream(first.begin() + first.size() / 2, first.end());
    for (char c : std::string("ESP-ROM:esp32s3-20210327\r\n")) stream.push_back(c);
    stream.insert(stream.end(), second.begin(), second.end());
    stream.insert(stream.end(), first.begin(), first.end());
    TEST_ASSERT_TRUE((decode(stream, 3) == std::vector<Bytes>{b, a}));
}

// A short write on the robot cuts a frame off; the frame after it must still arrive.
void test_a_truncated_frame_costs_only_itself() {
    const Bytes a = {1, 2, 3, 4, 5, 6}, b = {9, 9};
    Bytes stream = serial_frame::encode(a.data(), a.size());
    stream.resize(stream.size() - 3);
    const Bytes second = serial_frame::encode(b.data(), b.size());
    stream.insert(stream.end(), second.begin(), second.end());
    TEST_ASSERT_TRUE((decode(stream, 5) == std::vector<Bytes>{b}));
}

void test_a_message_over_the_limit_is_not_encoded() {
    const Bytes big(serial_frame::MAX_MESSAGE + 1, 1);
    TEST_ASSERT_TRUE(serial_frame::encode(big.data(), big.size()).empty());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_every_vector_decodes_alike_in_any_chunking);
    RUN_TEST(test_a_single_frame_encodes_to_its_vector);
    RUN_TEST(test_a_reader_starting_mid_frame_finds_the_next_frames);
    RUN_TEST(test_a_truncated_frame_costs_only_itself);
    RUN_TEST(test_a_message_over_the_limit_is_not_encoded);
    return UNITY_END();
}
