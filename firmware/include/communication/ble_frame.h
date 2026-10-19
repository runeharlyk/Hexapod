#pragma once

#include <cstddef>
#include <cstdint>

// Robot-to-app BLE framing: [uint16 LE length][payload], cut into notifications of CHUNK bytes and a shorter last one.
// Every frame starts a notification. The app reassembles a frame by its length prefix and checks that each chunk
// fits the frame it continues (ble-framing.ts), so a frame must reach it whole, or not at all.
namespace ble_frame {

constexpr size_t CHUNK = 180;
// A chunk after the first that finds no mbuf is retried this often, after a pause each: the controller frees mbufs
// as it transmits, at every connection event (15 to 30 ms apart), and notify() itself waits 10 ms for one.
constexpr size_t CHUNK_RETRIES = 5;

/**
 * Sends one frame, a notification per chunk, through `notify(bytes, len) -> bool`. A first chunk that fails loses the
 * frame whole and keeps the link. A later chunk is retried, `pause()` before each retry; one that never goes out would
 * leave the app with a frame cut short, which it would complete with the next frames' bytes, so `endLink()` ends the
 * link instead and the app reconnects with an empty buffer. True only when the frame went out whole.
 */
template <typename Notify, typename Pause, typename EndLink>
bool send(const uint8_t* data, size_t len, Notify&& notify, Pause&& pause, EndLink&& endLink) {
    uint8_t chunk[CHUNK];
    const size_t total = 2 + len;
    for (size_t at = 0; at < total;) {
        const size_t take = total - at < CHUNK ? total - at : CHUNK;
        for (size_t i = 0; i < take; i++) {
            const size_t pos = at + i;
            chunk[i] = pos == 0 ? (uint8_t)(len & 0xFF) : pos == 1 ? (uint8_t)((len >> 8) & 0xFF) : data[pos - 2];
        }
        bool sent = notify(chunk, take);
        if (!sent && at == 0) return false;
        for (size_t retry = 0; !sent && retry < CHUNK_RETRIES; retry++) {
            pause();
            sent = notify(chunk, take);
        }
        if (!sent) {
            endLink();
            return false;
        }
        at += take;
    }
    return true;
}

}  // namespace ble_frame
