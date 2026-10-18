#pragma once

// Chunked file transfer for transports without HTTP: the app on its https origin only has BLE and
// Web Serial. Each chunk is a self-contained stdio operation. Chunks land in `<path>.part`, and the
// chunk that completes the declared total replaces the file, so an upload cut off halfway leaves the
// previous file intact. Any status other than 200 on a chunk means the client restarts the file from
// offset 0: a failed fwrite can leave a partial chunk behind. Paths are checked by the caller against
// the mount root with validPath().

#include <sys/stat.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <filesystem.h>

namespace file_transfer {

constexpr size_t CHUNK_MAX = 512;

// Settings are reached only through their services: a raw read would leak the passwords those
// redact, and a raw write would bypass their validators.
constexpr char PRIVATE_DIRECTORY[] = "config";

// The filesystem skips repeated slashes and "." segments, so "//config" and "/./config" name the
// same directory and are matched here as well.
inline bool underPrivateDirectory(const char *rel) {
    const size_t nameLength = sizeof(PRIVATE_DIRECTORY) - 1;
    for (const char *segment = rel; *segment;) {
        while (*segment == '/') ++segment;
        const size_t length = strcspn(segment, "/");
        if (length == 1 && segment[0] == '.') {
            segment += length;
            continue;
        }
        return length == nameLength && strncmp(segment, PRIVATE_DIRECTORY, nameLength) == 0;
    }
    return false;
}

inline bool validPath(const char *rel) {
    return rel && rel[0] == '/' && strstr(rel, "..") == nullptr && !underPrivateDirectory(rel);
}

inline long fileSize(const char *full) {
    struct stat st;
    return stat(full, &st) == 0 ? (long)st.st_size : -1;
}

inline std::string partPath(const char *full) { return std::string(full) + ".part"; }

// 200 written; 400 the chunk does not fit the declared total, does not continue the upload, or is
// too large; 500 the filesystem refused.
inline int write(const char *full, uint32_t offset, uint32_t total, const uint8_t *data, size_t len) {
    if (len > CHUNK_MAX || (uint64_t)offset + len > total) return 400;
    const std::string part = partPath(full);
    if (offset != 0 && fileSize(part.c_str()) != (long)offset) return 400;
    FILE *f = fopen(part.c_str(), offset == 0 ? "wb" : "ab");
    if (!f) return 500;
    const bool ok = fwrite(data, 1, len, f) == len;
    fclose(f);
    if (!ok) return 500;
    if (offset + len < total) return 200;
    return FileSystem::replaceFile(part.c_str(), full) ? 200 : 500;
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
