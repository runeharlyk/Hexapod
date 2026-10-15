#pragma once

#include <peripherals/drivers/pca9685.h>
#include <cstdint>

/**
 * Consecutive failed writes to one board, and when to try it again. A board that stops answering costs
 * every write a NACK and the driver's two error log lines, or the whole transfer timeout while it holds a
 * line, on every 5 ms tick: after FAILURES_BEFORE_BACKOFF failed writes in a row it is tried only every
 * RETRY_INTERVAL_US, and on every tick again once it answers.
 */
class WriteBackoff {
  public:
    static constexpr uint32_t FAILURES_BEFORE_BACKOFF = 3;
    static constexpr int64_t RETRY_INTERVAL_US = 500000;

    bool due(int64_t nowUs) const {
        return _failures < FAILURES_BEFORE_BACKOFF || nowUs - _lastAttemptUs >= RETRY_INTERVAL_US;
    }

    void record(bool ok, int64_t nowUs) {
        _lastAttemptUs = nowUs;
        _failures = ok ? 0 : _failures + 1;
    }

    bool failing() const { return _failures > 0; }
    bool backingOff() const { return _failures >= FAILURES_BEFORE_BACKOFF; }

  private:
    uint32_t _failures = 0;
    int64_t _lastAttemptUs = 0;
};

/**
 * One PCA9685 and its backoff, so a board that stops answering neither stalls the control loop nor stops
 * the other board's writes. A board that answers again after failing may have lost power: then it answers
 * asleep, at its power-on frequency and without auto-increment, and the write that reached it landed in
 * one register, so it is started again. A board that did not answer at start counts as failing, so it is
 * started when it first answers.
 */
class ServoBoard {
  public:
    ServoBoard(uint8_t addr, uint32_t oscillatorHz, float pwmHz)
        : _pca(addr), _addr(addr), _oscillatorHz(oscillatorHz), _pwmHz(pwmHz) {}

    bool begin(int64_t nowUs) {
        const bool ok = _pca.begin(_oscillatorHz, _pwmHz);
        if (!ok) _backoff.record(false, nowUs);
        return ok;
    }

    void wakeup() {
        _awake = true;
        _pca.wakeup();
    }

    void sleep() {
        _awake = false;
        _pca.sleep();
    }

    // One tick's write of the first `count` channels. False when it failed, and when the board is backed
    // off and the write was skipped.
    bool write(const uint16_t* pwm, uint8_t count, int64_t nowUs) {
        if (!_backoff.due(nowUs)) return false;
        bool ok = _pca.setMultiplePWM(pwm, count) == 0;
        if (ok && _backoff.failing() && _pca.lostConfiguration()) ok = restart(pwm, count);
        _backoff.record(ok, nowUs);
        return ok;
    }

    // A single channel, as the calibration page sets it; not backed off, since a person drives it.
    void setPWM(uint8_t channel, uint16_t value) { _pca.setPWM(channel, 0, value); }

    uint8_t address() const { return _addr; }
    bool answering() const { return !_backoff.backingOff(); }

  private:
    // The pose goes into the channel registers before the chip wakes, so its outputs restart with it
    // rather than with the power-on full-off.
    bool restart(const uint16_t* pwm, uint8_t count) {
        if (!_pca.begin(_oscillatorHz, _pwmHz)) return false;
        const bool ok = _pca.setMultiplePWM(pwm, count) == 0;
        if (_awake) _pca.wakeup();
        return ok;
    }

    PCA9685Driver _pca;
    uint8_t _addr;
    uint32_t _oscillatorHz;
    float _pwmHz;
    bool _awake = false;
    WriteBackoff _backoff;
};
