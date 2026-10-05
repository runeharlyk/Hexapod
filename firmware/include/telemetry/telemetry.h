#pragma once

#include <atomic>
#include <platform_shared/message.pb.h>
#include <telemetry/spsc_ring.h>

// Hands tick samples from the control task to the service task. The control task records only while
// someone listens, never waits, and counts what a full ring refuses; the service task takes whole
// batches, so a recording sees every tick or knows how many it missed.
class Telemetry {
  public:
    static constexpr size_t RING_TICKS = 128;
    static constexpr size_t BATCH_TICKS = sizeof(socket_message_TelemetryBatch::ticks) / sizeof(socket_message_TickSample);

    void setRecording(bool recording) { _recording.store(recording, std::memory_order_relaxed); }
    bool recording() const { return _recording.load(std::memory_order_relaxed); }

    void record(const socket_message_TickSample& tick) {
        if (!recording()) return;
        if (!_ring.push(tick)) _dropped.fetch_add(1, std::memory_order_relaxed);
    }

    bool takeBatch(socket_message_TelemetryBatch& batch) {
        if (_ring.size() < BATCH_TICKS) return false;
        batch = socket_message_TelemetryBatch_init_zero;
        batch.batch_seq = _batchSeq++;
        batch.dropped_ticks = droppedTicks();
        while (batch.ticks_count < BATCH_TICKS && _ring.pop(batch.ticks[batch.ticks_count])) batch.ticks_count++;
        return true;
    }

    // Called by the service task while nobody listens, so a new recording starts with fresh ticks.
    void discardUnsent() { _ring.discard(); }

    uint32_t droppedTicks() const { return _dropped.load(std::memory_order_relaxed); }

  private:
    SpscRing<socket_message_TickSample, RING_TICKS> _ring;
    std::atomic<bool> _recording{false};
    std::atomic<uint32_t> _dropped{0};
    uint32_t _batchSeq = 0;
};
