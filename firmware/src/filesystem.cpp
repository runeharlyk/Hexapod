#include <filesystem.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>

#include <esp_littlefs.h>
#include <esp_log.h>

static const char *TAG = "FileSystem";

namespace FileSystem {

bool mkdirRecursive(const char *path) {
    char buf[256];
    strncpy(buf, path, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    for (char *p = buf + 1; *p; ++p) {
        if (*p != '/') continue;
        *p = '\0';
        if (mkdir(buf, 0775) != 0 && errno != EEXIST) {
            ESP_LOGE(TAG, "mkdir %s failed (errno %d)", buf, errno);
            return false;
        }
        *p = '/';
    }
    if (mkdir(buf, 0775) != 0 && errno != EEXIST) {
        ESP_LOGE(TAG, "mkdir %s failed (errno %d)", buf, errno);
        return false;
    }
    return true;
}

bool init() {
    esp_vfs_littlefs_conf_t conf = {
        .base_path = MOUNT_POINT,
        .partition_label = FS_PARTITION_LABEL,
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount LittleFS (%s)", esp_err_to_name(ret));
        return false;
    }

    size_t total = 0, used = 0;
    if (esp_littlefs_info(FS_PARTITION_LABEL, &total, &used) == ESP_OK) {
        ESP_LOGI(TAG, "LittleFS mounted at %s: %u/%u bytes used", MOUNT_POINT, (unsigned)used, (unsigned)total);
    }

    mkdirRecursive(FS_CONFIG_DIRECTORY);
    return true;
}

bool readFile(const char *path, std::string &out) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    char buf[512];
    size_t n;
    out.clear();
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    fclose(f);
    return true;
}

bool writeFile(const char *path, const char *content) {
    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s for writing", path);
        return false;
    }
    size_t len = strlen(content);
    bool ok = fwrite(content, 1, len, f) == len;
    fclose(f);
    return ok;
}

} // namespace FileSystem
