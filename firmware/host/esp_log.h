#pragma once

// Host stand-in for ESP-IDF's esp_log.h: errors and warnings go to stderr, the rest is dropped.

#include <cstdio>

#define ESP_LOGE(tag, fmt, ...) std::fprintf(stderr, "E %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) std::fprintf(stderr, "W %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)
