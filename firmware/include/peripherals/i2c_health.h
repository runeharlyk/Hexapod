#pragma once

// The hardware-free half of the I2C bus's error handling: how a failed transfer is classified and
// counted, when a held bus is clocked free, and the bus-clear sequence itself. I2CBus applies it to
// the ESP-IDF driver and the pins; the host tests run it against fakes.

#include <cstddef>
#include <cstdint>

namespace i2c_health {

enum class Failure { NACK, TIMEOUT };

/**
 * The IDF 5.4 master driver fails a NACK and a timeout alike, with ESP_ERR_INVALID_STATE, so the bus
 * tells them apart itself: a timeout leaves a line held low or spends the transfer's whole time, which
 * a device that does not acknowledge does not. The driver waits whole ticks, so a timeout can end one
 * tick short of the timeout.
 */
inline Failure classify(bool driverTimedOut, int64_t elapsedUs, int timeoutMs, int tickMs, bool sdaLow, bool sclLow) {
    const bool spentTimeout = elapsedUs >= static_cast<int64_t>(timeoutMs - tickMs) * 1000;
    return driverTimedOut || spentTimeout || sdaLow || sclLow ? Failure::TIMEOUT : Failure::NACK;
}

struct DeviceErrors {
    uint8_t address = 0;
    uint32_t nacks = 0;
    uint32_t timeouts = 0;
};

// Failed transfers per device since boot, in address order. A device is listed from its first
// transfer, so one that always answers shows with no errors; a full table ignores further addresses.
template <size_t Capacity>
class ErrorTable {
  public:
    void track(uint8_t address) { find(address); }

    void count(uint8_t address, Failure failure) {
        DeviceErrors* device = find(address);
        if (!device) return;
        failure == Failure::NACK ? device->nacks++ : device->timeouts++;
    }

    size_t size() const { return _size; }
    const DeviceErrors& operator[](size_t i) const { return _devices[i]; }

  private:
    DeviceErrors* find(uint8_t address) {
        size_t i = 0;
        while (i < _size && _devices[i].address < address) i++;
        if (i < _size && _devices[i].address == address) return &_devices[i];
        if (_size == Capacity) return nullptr;
        for (size_t j = _size; j > i; j--) _devices[j] = _devices[j - 1];
        _devices[i] = DeviceErrors {address, 0, 0};
        _size++;
        return &_devices[i];
    }

    DeviceErrors _devices[Capacity];
    size_t _size = 0;
};

/**
 * A device cut off mid-read, by a reset of the ESP32 or a glitch, holds SDA low until it has clocked out
 * its byte. Clocking frees only that case: a held SCL is a device stretching the clock, or one without
 * power clamping both lines. A recovery restarts the bus, which costs more than a transfer, so a device
 * that never lets go is tried at most once a second rather than on every failed transfer.
 */
class RecoveryLimiter {
  public:
    static constexpr int64_t INTERVAL_US = 1000000;

    bool due(bool sdaLow, bool sclLow, int64_t nowUs) {
        if (!sdaLow || sclLow || nowUs < _nextUs) return false;
        _nextUs = nowUs + INTERVAL_US;
        _count++;
        return true;
    }

    // Recoveries since boot.
    uint32_t count() const { return _count; }

  private:
    int64_t _nextUs = 0;
    uint32_t _count = 0;
};

/**
 * The standard bus clear on open-drain pins the caller has released high: up to nine SCL pulses, until
 * the device lets SDA go, then a STOP. SDA changes only while SCL is low, except for the STOP itself,
 * so the sequence never puts a START on the bus. Returns the pulses given.
 *
 * Pins provides sda(bool high), scl(bool high), sdaLow() and halfPeriod().
 */
template <typename Pins>
int clockOut(Pins& pins) {
    constexpr int MAX_PULSES = 9;
    int pulses = 0;
    for (; pulses < MAX_PULSES && pins.sdaLow(); pulses++) {
        pins.scl(false);
        pins.halfPeriod();
        pins.scl(true);
        pins.halfPeriod();
    }
    pins.scl(false);
    pins.sda(false);
    pins.halfPeriod();
    pins.scl(true);
    pins.halfPeriod();
    pins.sda(true);
    pins.halfPeriod();
    return pulses;
}

} // namespace i2c_health
