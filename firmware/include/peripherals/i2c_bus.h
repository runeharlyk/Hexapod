#pragma once

#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <functional>
#include <vector>
#include <cstring>

class I2CBus {
    static constexpr const char* TAG = "I2CBus";
    static constexpr TickType_t TRANSFER_TIMEOUT = pdMS_TO_TICKS(200);
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
        Lock lock(_mutex);
        if (!lock.held()) return ESP_ERR_TIMEOUT;
        if (!_initialized) return ESP_ERR_INVALID_STATE;

        i2c_master_dev_handle_t dev = NULL;
        esp_err_t err = ensureDevice(addr, dev);
        if (err != ESP_OK) return err;
        return i2c_master_transmit(dev, data, len, TRANSFER_TIMEOUT);
    }

    esp_err_t writeReg(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
        // Bounded stack buffer (no VLA); 64 covers the largest write (PCA9685).
        if (len > 64) return ESP_ERR_INVALID_SIZE;

        Lock lock(_mutex);
        if (!lock.held()) return ESP_ERR_TIMEOUT;
        if (!_initialized) return ESP_ERR_INVALID_STATE;

        i2c_master_dev_handle_t dev = NULL;
        esp_err_t err = ensureDevice(addr, dev);
        if (err != ESP_OK) return err;

        uint8_t buf[65];
        buf[0] = reg;
        if (len > 0 && data != nullptr) {
            memcpy(buf + 1, data, len);
        }
        return i2c_master_transmit(dev, buf, len + 1, TRANSFER_TIMEOUT);
    }

    esp_err_t readReg(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
        Lock lock(_mutex);
        if (!lock.held()) return ESP_ERR_TIMEOUT;
        if (!_initialized) return ESP_ERR_INVALID_STATE;

        i2c_master_dev_handle_t dev = NULL;
        esp_err_t err = ensureDevice(addr, dev);
        if (err != ESP_OK) return err;
        return i2c_master_transmit_receive(dev, &reg, 1, data, len, TRANSFER_TIMEOUT);
    }

    bool probe(uint8_t addr) {
        Lock lock(_mutex);
        if (!lock.held()) return false;
        if (!_initialized) return false;

        return i2c_master_probe(_bus, addr, TRANSFER_TIMEOUT) == ESP_OK;
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
    static constexpr size_t MAX_DEVICES = 8;
    Device _devices[MAX_DEVICES];
    size_t _nextEviction = 0;

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
