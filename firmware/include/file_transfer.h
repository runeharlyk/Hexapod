#pragma once

// Chunked file transfer for transports without HTTP: the app on its https origin only has BLE and
// Web Serial. Each chunk is a self-contained stdio operation, so a lost chunk leaves a file that is
// simply shorter than its total; the next write at the right offset continues it. Paths are checked
// by the caller against the mount root with validPath().

#include <sys/stat.h>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace file_transfer {

constexpr size_t CHUNK_MAX = 512;

inline bool validPath(const char *rel) { return rel && rel[0] == '/' && strstr(rel, "..") == nullptr; }

inline long fileSize(const char *full) {
    struct stat st;
    return stat(full, &st) == 0 ? (long)st.st_size : -1;
}

// 200 written; 400 the chunk does not fit the declared total, does not continue the file, or is
// too large; 500 the filesystem refused.
inline int write(const char *full, uint32_t offset, uint32_t total, const uint8_t *data, size_t len) {
    if (len > CHUNK_MAX || (uint64_t)offset + len > total) return 400;
    if (offset == 0) {
        FILE *f = fopen(full, "wb");
        if (!f) return 500;
        const bool ok = fwrite(data, 1, len, f) == len;
        fclose(f);
        return ok ? 200 : 500;
    }
    if (fileSize(full) != (long)offset) return 400;
    FILE *f = fopen(full, "ab");
    if (!f) return 500;
    const bool ok = fwrite(data, 1, len, f) == len;
    fclose(f);
    return ok ? 200 : 500;
}

// 200 with n bytes and the file's total; 404 no such file; 400 length over CHUNK_MAX.
inline int read(const char *full, uint32_t offset, uint32_t length, uint8_t *out, size_t &n, uint32_t &total) {
    n = 0;
    if (length > CHUNK_MAX) return 400;
    const long size = fileSize(full);
    if (size < 0) return 404;
    total = (uint32_t)size;
    FILE *f = fopen(full, "rb");
    if (!f) return 404;
    if (fseek(f, (long)offset, SEEK_SET) == 0) n = fread(out, 1, length, f);
    fclose(f);
    return 200;
}

}  // namespace file_transfer
