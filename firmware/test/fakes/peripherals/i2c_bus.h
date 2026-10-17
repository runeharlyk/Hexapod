#pragma once

// Host stand-in for firmware/include/peripherals/i2c_bus.h, found first on the native include path.
// Each device is a 256-byte register file; every write is recorded so a test can check the order in
// which a driver touched the chip, not only where it left it. A device answers while it has registers
// and is not marked absent, so a test can unplug a chip and plug it back with its registers intact. Each transfer,
// answered or not, takes transferUs of the host clock, so a test can time what a driver spends on the bus.

#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <vector>
#include <host_clock.h>

using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_FAIL = -1;

using TickType_t = uint32_t;
#define pdMS_TO_TICKS(ms) static_cast<TickType_t>(ms)
inline void vTaskDelay(TickType_t) {}
enum gpio_num_t : int {};

struct FakeWrite {
    uint8_t addr;
    uint8_t reg;
    std::vector<uint8_t> bytes;
};

class I2CBus {
  public:
    static I2CBus& instance() {
        static I2CBus inst;
        return inst;
    }

    void reset() {
        registers.clear();
        writes.clear();
        absent.clear();
        attempts.clear();
        transferUs = 0;
    }

    // The bus is up before any driver touches it, so the servo controller's start-up has nothing to do.
    bool isInitialized() const { return true; }
    void begin(gpio_num_t, gpio_num_t, uint32_t) {}

    bool probe(uint8_t addr) { return registers.count(addr) != 0 && absent.count(addr) == 0; }

    esp_err_t writeReg(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
        transfer(addr);
        if (!probe(addr)) return ESP_FAIL;
        writes.push_back({addr, reg, std::vector<uint8_t>(data, data + len)});
        // Auto-increment, as on the PCA9685 with MODE1.AI set.
        for (size_t i = 0; i < len; i++) registers[addr][(reg + i) & 0xFF] = data[i];
        return ESP_OK;
    }

    esp_err_t readReg(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
        transfer(addr);
        if (!probe(addr)) return ESP_FAIL;
        for (size_t i = 0; i < len; i++) data[i] = registers[addr][(reg + i) & 0xFF];
        return ESP_OK;
    }

    std::map<uint8_t, std::array<uint8_t, 256>> registers;
    std::vector<FakeWrite> writes;
    std::set<uint8_t> absent;
    // Register transfers addressed to each device, answered or not.
    std::map<uint8_t, int> attempts;
    int64_t transferUs = 0;

  private:
    void transfer(uint8_t addr) {
        attempts[addr]++;
        host_clock::advance(transferUs);
    }
};
