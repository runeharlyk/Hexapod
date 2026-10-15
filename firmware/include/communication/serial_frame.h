#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

// Protobuf messages on the native USB port. A frame is 0x00, COBS(version, message, CRC-16), 0x00:
// the delimiters let a reader that starts mid-stream, or loses bytes to a short write, find the next
// frame, and the CRC drops a damaged one instead of decoding garbage. Bytes outside a valid frame
// (the ROM's boot banner after a reset) are dropped. The app's serial-framing.ts implements the same,
// and platform_shared/serial_frame_vectors.json pins both.
namespace serial_frame {

constexpr uint8_t VERSION = 1;
constexpr size_t MAX_MESSAGE = 4096;
// Version, message and CRC, plus a COBS code byte for every 254 bytes and one more.
constexpr size_t MAX_ENCODED = (1 + MAX_MESSAGE + 2) + (1 + MAX_MESSAGE + 2) / 254 + 1;

// CRC-16/CCITT-FALSE: polynomial 0x1021, initial 0xFFFF, not reflected.
inline uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; bit++) crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

// The bytes to write for one message, delimiters included; empty for a message over MAX_MESSAGE.
inline std::vector<uint8_t> encode(const uint8_t* message, size_t len) {
    if (len > MAX_MESSAGE) return {};
    std::vector<uint8_t> body;
    body.reserve(len + 3);
    body.push_back(VERSION);
    body.insert(body.end(), message, message + len);
    const uint16_t crc = crc16(body.data(), body.size());
    body.push_back(crc >> 8);
    body.push_back(crc & 0xFF);

    std::vector<uint8_t> out;
    out.reserve(body.size() + body.size() / 254 + 3);
    out.push_back(0);
    size_t codeAt = out.size();
    out.push_back(1);
    for (uint8_t byte : body) {
        if (byte != 0) {
            out.push_back(byte);
            out[codeAt]++;
        }
        if (byte == 0 || out[codeAt] == 0xFF) {
            codeAt = out.size();
            out.push_back(1);
        }
    }
    out.push_back(0);
    return out;
}

// Splits a byte stream, fed in pieces of any size, into messages. A segment that is not a valid frame
// is dropped, and the 0x00 that ended it starts the next frame.
class Decoder {
  public:
    using FrameHandler = std::function<void(const uint8_t* message, size_t len)>;

    explicit Decoder(FrameHandler onFrame) : _onFrame(std::move(onFrame)) {}

    void feed(const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; i++) feed(data[i]);
    }

    void reset() {
        _inFrame = false;
        _segment.clear();
    }

  private:
    FrameHandler _onFrame;
    bool _inFrame = false;
    std::vector<uint8_t> _segment;

    void feed(uint8_t byte) {
        if (!_inFrame) {
            if (byte == 0) {
                _inFrame = true;
                _segment.clear();
            }
            return;
        }
        if (byte != 0) {
            _segment.push_back(byte);
            if (_segment.size() > MAX_ENCODED) reset();
            return;
        }
        if (_segment.empty()) return;
        // A delimiter that ends no valid frame may open the next one, as for a reader that started
        // mid-frame; one that ended a frame closes it.
        if (deliver()) _inFrame = false;
        _segment.clear();
    }

    bool deliver() {
        std::vector<uint8_t> body;
        body.reserve(_segment.size());
        for (size_t i = 0; i < _segment.size();) {
            const uint8_t code = _segment[i++];
            if (code == 0 || i + code - 1 > _segment.size()) return false;
            body.insert(body.end(), _segment.begin() + i, _segment.begin() + i + code - 1);
            i += code - 1;
            if (code != 0xFF && i < _segment.size()) body.push_back(0);
        }
        if (body.size() < 3 || body[0] != VERSION || body.size() - 3 > MAX_MESSAGE) return false;
        const uint16_t crc = static_cast<uint16_t>(body[body.size() - 2] << 8 | body[body.size() - 1]);
        if (crc16(body.data(), body.size() - 2) != crc) return false;
        _onFrame(body.data() + 1, body.size() - 3);
        return true;
    }
};

}  // namespace serial_frame
