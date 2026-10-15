#pragma once

#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#include <esp_log.h>
#include <soc/soc_caps.h>
#include <peripherals/status_led_pattern.h>

/**
 * One WS2812 on an RMT channel. The IDF's bytes encoder turns the three colour bytes into the LED's
 * pulse widths, so no LED-strip component or pixel buffer is needed: one channel's own RMT memory, no
 * DMA, and some hundreds of bytes of driver state in internal RAM. The LED holds its colour, so a
 * colour is sent only when it changes.
 */
class StatusLed {
  public:
    bool begin(gpio_num_t pin) {
        rmt_tx_channel_config_t channel = {};
        channel.gpio_num = pin;
        channel.clk_src = RMT_CLK_SRC_DEFAULT;
        channel.resolution_hz = RESOLUTION_HZ;
        channel.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
        channel.trans_queue_depth = 1;
        esp_err_t err = rmt_new_tx_channel(&channel, &_channel);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "No RMT channel for the status LED: %s", esp_err_to_name(err));
            _channel = nullptr;
            return false;
        }

        // WS2812 bit timing at 10 MHz: a 0 is high 0.3 us then low 0.9 us, a 1 the reverse; MSB first.
        rmt_bytes_encoder_config_t bits = {};
        bits.bit0.duration0 = 3;
        bits.bit0.level0 = 1;
        bits.bit0.duration1 = 9;
        bits.bit0.level1 = 0;
        bits.bit1.duration0 = 9;
        bits.bit1.level0 = 1;
        bits.bit1.duration1 = 3;
        bits.bit1.level1 = 0;
        bits.flags.msb_first = 1;
        err = rmt_new_bytes_encoder(&bits, &_encoder);
        if (err == ESP_OK) err = rmt_enable(_channel);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Status LED not started: %s", esp_err_to_name(err));
            if (_encoder) rmt_del_encoder(_encoder);
            rmt_del_channel(_channel);
            _encoder = nullptr;
            _channel = nullptr;
            return false;
        }
        return true;
    }

    void show(status_led::Rgb colour) {
        if (!_channel || (_shown && colour == _last)) return;
        // The frame must outlive its 30 us transmission, so one still leaving from the last call (its
        // interrupt waits out a flash write) is waited for before the frame changes.
        if (rmt_tx_wait_all_done(_channel, WAIT_MS) != ESP_OK) return;
        // The WS2812 takes green, red, blue.
        _frame[0] = colour.g;
        _frame[1] = colour.r;
        _frame[2] = colour.b;
        rmt_transmit_config_t transmit = {};
        if (rmt_transmit(_channel, _encoder, _frame, sizeof(_frame), &transmit) != ESP_OK) return;
        _last = colour;
        _shown = true;
    }

  private:
    static constexpr const char* TAG = "StatusLed";
    static constexpr uint32_t RESOLUTION_HZ = 10000000;
    static constexpr int WAIT_MS = 10;

    rmt_channel_handle_t _channel = nullptr;
    rmt_encoder_handle_t _encoder = nullptr;
    uint8_t _frame[3] = {};
    status_led::Rgb _last;
    bool _shown = false;
};
