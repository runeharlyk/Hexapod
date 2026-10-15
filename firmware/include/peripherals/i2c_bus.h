#pragma once

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <functional>
#include <vector>
#include <cstring>
#include <peripherals/i2c_health.h>

class I2CBus {
    static constexpr const char* TAG = "I2CBus";
    // The i2c_master driver takes milliseconds, not ticks. Long enough for any transfer here (a 64-byte
    // PCA9685 write takes under 6 ms at 100 kHz); short, because a held line costs the 5 ms control
    // loop this long on every transfer until the bus is free again.
    static constexpr int TRANSFER_TIMEOUT_MS = 20;
    static constexpr TickType_t LOCK_TIMEOUT = pdMS_TO_TICKS(50);

    // Bounded acquisition: a caller that cannot get the bus fails its transfer instead of blocking the
    // 5 ms control loop. Recursive because begin() -> end() and scan() -> probe() re-enter.
    class Lock {
      public:
        explicit Lock(SemaphoreHandle_t mutex)
            : _mutex(mutex), _held(xSemaphoreTakeRecursive(mutex, LOCK_TIMEOUT) == pdTRUE) {}
        ~Lock() {
            if (_held) xSemaphoreGiveRecursive(_mutex);
        }
        Lock(const Lock&) = delete;
        Lock& operator=(const Lock&) = delete;

        bool held() const { return _held; }

      private:
        SemaphoreHandle_t _mutex;
        bool _held;
    };

  public:
    static I2CBus& instance() {
        static I2CBus inst;
        return inst;
    }

    esp_err_t begin(gpio_num_t sda, gpio_num_t scl, uint32_t freq = 100000, i2c_port_t port = I2C_NUM_0) {
        Lock lock(_mutex);
        if (!lock.held()) return ESP_ERR_TIMEOUT;

        if (_initialized) {
            end();
        }

        _port = port;
        _sda = sda;
        _scl = scl;
        _freq = freq;

        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = port;
        bus_cfg.sda_io_num = sda;
        bus_cfg.scl_io_num = scl;
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
#if CONFIG_IDF_TARGET_ESP32P4
        bus_cfg.flags.enable_internal_pullup = false;
#else
        bus_cfg.flags.enable_internal_pullup = true;
#endif

        esp_err_t err = i2c_new_master_bus(&bus_cfg, &_bus);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
            return err;
        }

        _initialized = true;
        return ESP_OK;
    }

    void end() {
        Lock lock(_mutex);
        if (!lock.held()) return;

        if (_initialized) {
            for (Device& d : _devices) {
                if (d.handle) i2c_master_bus_rm_device(d.handle);
                d = Device{};
            }
            i2c_del_master_bus(_bus);
            _bus = NULL;
            _initialized = false;
        }
    }

    bool isInitialized() const { return _initialized; }


    esp_err_t writeBytes(uint8_t addr, const uint8_t* data, size_t len) {
        return transfer(addr, [&](i2c_master_dev_handle_t dev) {
            return i2c_master_transmit(dev, data, len, TRANSFER_TIMEOUT_MS);
        });
    }

    esp_err_t writeReg(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
        // Bounded stack buffer (no VLA); 64 covers the largest write (PCA9685).
        if (len > 64) return ESP_ERR_INVALID_SIZE;

        uint8_t buf[65];
        buf[0] = reg;
        if (len > 0 && data != nullptr) {
            memcpy(buf + 1, data, len);
        }
        return transfer(addr, [&](i2c_master_dev_handle_t dev) {
            return i2c_master_transmit(dev, buf, len + 1, TRANSFER_TIMEOUT_MS);
        });
    }

    esp_err_t readReg(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
        return transfer(addr, [&](i2c_master_dev_handle_t dev) {
            return i2c_master_transmit_receive(dev, &reg, 1, data, len, TRANSFER_TIMEOUT_MS);
        });
    }

    bool probe(uint8_t addr) {
        Lock lock(_mutex);
        if (!lock.held()) return false;
        if (!_initialized) return false;

        return i2c_master_probe(_bus, addr, TRANSFER_TIMEOUT_MS) == ESP_OK;
    }

    // Deliberately unlocked: each probe() takes the lock on its own so a 127-address sweep from the
    // service task cannot starve the 5 ms control loop for the whole scan.
    std::vector<uint8_t> scan(uint8_t lower = 1, uint8_t upper = 127) {
        std::vector<uint8_t> devices;
        if (!_initialized) return devices;

        for (uint8_t addr = lower; addr < upper; addr++) {
            if (probe(addr)) {
                devices.push_back(addr);
                ESP_LOGI(TAG, "I2C device found at address 0x%02X", addr);
            }
        }
        ESP_LOGI(TAG, "Scan complete - Found %zu device(s)", devices.size());
        return devices;
    }

    static constexpr size_t MAX_DEVICES = 8;

    // Copies the failed transfers of every device the bus has transferred with since boot, in address
    // order, and returns how many; none when the bus stays busy past the lock timeout.
    size_t errors(i2c_health::DeviceErrors* out, size_t capacity) {
        Lock lock(_mutex);
        if (!lock.held()) return 0;
        const size_t n = _errors.size() < capacity ? _errors.size() : capacity;
        for (size_t i = 0; i < n; i++) out[i] = _errors[i];
        return n;
    }

    // Times since boot the bus clocked free a device that held SDA low.
    uint32_t recoveries() {
        Lock lock(_mutex);
        return lock.held() ? _recovery.count() : 0;
    }

  private:
    I2CBus() : _mutex(xSemaphoreCreateRecursiveMutex()) {}
    ~I2CBus() {
        end();
        vSemaphoreDelete(_mutex);
    }
    I2CBus(const I2CBus&) = delete;
    I2CBus& operator=(const I2CBus&) = delete;

    SemaphoreHandle_t _mutex;
    i2c_port_t _port = I2C_NUM_0;
    gpio_num_t _sda = GPIO_NUM_NC;
    gpio_num_t _scl = GPIO_NUM_NC;
    uint32_t _freq = 100000;
    bool _initialized = false;

    i2c_master_bus_handle_t _bus = NULL;

    // One handle per address, added on first use and kept until the bus closes: re-adding a device on
    // every address switch cost several heap allocations per 5 ms control tick.
    struct Device {
        uint8_t addr = 0xFF;
        i2c_master_dev_handle_t handle = NULL;
    };
    Device _devices[MAX_DEVICES];
    size_t _nextEviction = 0;
    // Kept apart from the device handles, which a restart of the bus drops: the counts are since boot.
    i2c_health::ErrorTable<MAX_DEVICES> _errors;
    i2c_health::RecoveryLimiter _recovery;

    template <typename Run>
    esp_err_t transfer(uint8_t addr, Run&& run) {
        Lock lock(_mutex);
        if (!lock.held()) return ESP_ERR_TIMEOUT;
        if (!_initialized) return ESP_ERR_INVALID_STATE;

        i2c_master_dev_handle_t dev = NULL;
        esp_err_t err = ensureDevice(addr, dev);
        if (err != ESP_OK) return err;
        _errors.track(addr);
        const int64_t start = esp_timer_get_time();
        err = run(dev);
        if (err != ESP_OK) afterFailure(addr, err, esp_timer_get_time() - start);
        return err;
    }

    // Called with the lock held.
    void afterFailure(uint8_t addr, esp_err_t err, int64_t elapsedUs) {
        const bool sdaLow = gpio_get_level(_sda) == 0;
        const bool sclLow = gpio_get_level(_scl) == 0;
        _errors.count(addr, i2c_health::classify(err == ESP_ERR_TIMEOUT, elapsedUs, TRANSFER_TIMEOUT_MS,
                                                 portTICK_PERIOD_MS, sdaLow, sclLow));
        if (_recovery.due(sdaLow, sclLow, esp_timer_get_time())) recover();
    }

    struct BusPins {
        gpio_num_t sdaPin;
        gpio_num_t sclPin;
        void sda(bool high) { gpio_set_level(sdaPin, high); }
        void scl(bool high) { gpio_set_level(sclPin, high); }
        bool sdaLow() const { return gpio_get_level(sdaPin) == 0; }
        void halfPeriod() { esp_rom_delay_us(5); }  // 100 kHz, the usual rate for a bus clear
    };

    /**
     * Clocks a device that holds SDA low free on the pins. The driver's own bus clear does not reach it on
     * the ESP32-S3: it starts the hardware's nine SCL pulses and resets the whole peripheral straight after.
     * Called with the lock held; end() and begin() re-enter it.
     */
    void recover() {
        const gpio_num_t sda = _sda, scl = _scl;
        const uint32_t freq = _freq;
        const i2c_port_t port = _port;
        end();
        BusPins pins{sda, scl};
        pins.sda(true);
        pins.scl(true);
        gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
        gpio_set_direction(scl, GPIO_MODE_INPUT_OUTPUT_OD);
        const int pulses = i2c_health::clockOut(pins);
        ESP_LOGW(TAG, "SDA was held low; clocked the bus free with %d pulse(s)", pulses);
        if (begin(sda, scl, freq, port) != ESP_OK) ESP_LOGE(TAG, "The bus did not start again after a recovery");
    }

    esp_err_t ensureDevice(uint8_t addr, i2c_master_dev_handle_t& out) {
        Device* slot = nullptr;
        for (Device& d : _devices) {
            if (d.handle && d.addr == addr) {
                out = d.handle;
                return ESP_OK;
            }
            if (!d.handle && !slot) slot = &d;
        }
        if (!slot) {
            // More distinct addresses than slots (e.g. a user-driven sweep); recycle one in turn.
            slot = &_devices[_nextEviction];
            _nextEviction = (_nextEviction + 1) % MAX_DEVICES;
            i2c_master_bus_rm_device(slot->handle);
            *slot = Device{};
        }
        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = addr;
        dev_cfg.scl_speed_hz = _freq;
        esp_err_t err = i2c_master_bus_add_device(_bus, &dev_cfg, &slot->handle);
        if (err != ESP_OK) {
            slot->handle = NULL;
            return err;
        }
        slot->addr = addr;
        out = slot->handle;
        return ESP_OK;
    }
};
