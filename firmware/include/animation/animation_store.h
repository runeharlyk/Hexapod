#pragma once

// Reads /littlefs/animations/<name>.pb into PSRAM, decodes it and validates it. The encoded and the
// decoded buffers are allocated once: a full clip decodes to about 5 KB and the encoded file is
// bounded by the schema, so neither belongs on a task stack or in internal RAM.

#include <dirent.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <sys/stat.h>
#include <cstdio>
#include <cstring>
#include <functional>
#include <pb_decode.h>
#include <animation/animation.h>
#include <animation/animation_codec.h>
#include <filesystem.h>

#define ANIMATION_DIRECTORY MOUNT_POINT "/animations"

class AnimationStore {
  public:
    static constexpr size_t ENCODED_MAX = animation_Animation_size;

    bool begin() {
        encoded_ = (uint8_t *)heap_caps_malloc(ENCODED_MAX, MALLOC_CAP_SPIRAM);
        decoded_ = (animation_Animation *)heap_caps_malloc(sizeof(animation_Animation), MALLOC_CAP_SPIRAM);
        if (!encoded_ || !decoded_) {
            ESP_LOGE(TAG, "no PSRAM for the animation buffers");
            return false;
        }
        FileSystem::mkdirRecursive(ANIMATION_DIRECTORY);
        return true;
    }

    static bool path(const char *name, char *out, size_t size) {
        if (!anim::validName(name)) return false;
        snprintf(out, size, ANIMATION_DIRECTORY "/%s.pb", name);
        return true;
    }

    // Decodes and validates; on failure `error` names the reason and `out` is unspecified.
    // Callers serialise: the adapter tasks and the event bus worker share the two buffers.
    bool load(const char *name, anim::Clip &out, const char *&error) {
        char file[96];
        if (!path(name, file, sizeof(file))) {
            error = "invalid name";
            return false;
        }
        FILE *f = fopen(file, "rb");
        if (!f) {
            error = "no such animation";
            return false;
        }
        const size_t n = fread(encoded_, 1, ENCODED_MAX, f);
        const bool more = fgetc(f) != EOF;
        fclose(f);
        if (more) {
            error = "file larger than the schema allows";
            return false;
        }
        // pb_decode sets every field absent from the stream to its default, so the buffer needs no reset.
        pb_istream_t stream = pb_istream_from_buffer(encoded_, n);
        if (!pb_decode(&stream, animation_Animation_fields, decoded_)) {
            error = PB_GET_ERROR(&stream);
            return false;
        }
        anim::fromProto(*decoded_, out);
        error = anim::validate(out);
        if (error) return false;
        if (strcmp(out.name, name) != 0) {
            error = "name does not match the file";
            return false;
        }
        return true;
    }

    // Calls fn(name, bytes) for every .pb in the directory whose stem fits a clip name.
    static void list(const std::function<void(const char *, uint32_t)> &fn) {
        DIR *dir = opendir(ANIMATION_DIRECTORY);
        if (!dir) return;
        for (struct dirent *e = readdir(dir); e; e = readdir(dir)) {
            const size_t len = strlen(e->d_name);
            if (len < 4 || strcmp(e->d_name + len - 3, ".pb") != 0) continue;
            const size_t stem = len - 3;
            if (stem > (size_t)anim::NAME_LEN_MAX) continue;
            char name[anim::NAME_LEN_MAX + 1] = {0};
            memcpy(name, e->d_name, stem);
            char full[96];
            snprintf(full, sizeof(full), ANIMATION_DIRECTORY "/%s", e->d_name);
            struct stat st;
            fn(name, stat(full, &st) == 0 ? (uint32_t)st.st_size : 0);
        }
        closedir(dir);
    }

  private:
    static constexpr const char *TAG = "AnimationStore";
    uint8_t *encoded_ = nullptr;
    animation_Animation *decoded_ = nullptr;
};
