#pragma once

// File access over the socket, on every transport: chunked transfer, which the app's https origin needs since it only
// has BLE and Web Serial, and directory listing and creation. Each chunk is a self-contained stdio operation. Chunks
// land in `<path>.part`, and the chunk that completes the declared total replaces the file, so an upload cut off
// halfway leaves the previous file intact. Any status other than 200 on a chunk means the client restarts the file
// from offset 0: a failed fwrite can leave a partial chunk behind. The caller resolves a chunk's path against the
// mount root with resolve(); list() and mkdir() resolve their own.

#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <filesystem.h>
#include <platform_shared/api.pb.h>

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

// Every request names its file by `rel`, which validPath() has to accept; the file system's own path is that
// under `root`.
inline bool resolve(const char *rel, std::string &full, const char *root = MOUNT_POINT) {
    if (!validPath(rel)) return false;
    full = std::string(root) + rel;
    return true;
}

namespace detail {

// The order of a listing: directories first, then each group in byte order of the name.
inline bool listedBefore(const api_FileEntry &a, const api_FileEntry &b) {
    return a.is_directory != b.is_directory ? a.is_directory : strcmp(a.name, b.name) < 0;
}

// `parent`, a separator unless it ends in one, then `name`; false when that does not fit `size`.
inline bool childPath(char *out, size_t size, const char *parent, const char *name) {
    const size_t length = strlen(parent);
    return snprintf(out, size, "%s%s%s", parent, length && parent[length - 1] == '/' ? "" : "/", name) < (int)size;
}

// Whether the entry at `full` is a directory: by its d_type, which esp_littlefs fills, or by stat() where the entry
// has none (the Windows C runtime's dirent) or leaves it DT_UNKNOWN.
template <typename Entry>
bool isDirectory(const Entry &e, const char *full) {
#ifdef DT_UNKNOWN
    if constexpr (requires { e.d_type; })
        if (e.d_type != DT_UNKNOWN) return e.d_type == DT_DIR;
#endif
    struct stat st;
    return stat(full, &st) == 0 && S_ISDIR(st.st_mode);
}

// One scan of a directory, keeping in `dest` (sorted, room for `count` >= 1) the entries that follow `after` in the
// listing's order, without their sizes; nothing but `dest` and two path buffers is held. `total` is every entry the
// listing counts. -1 when the directory cannot be opened, else the number of entries written to `dest`.
inline int scan(const char *full, const char *rel, const api_FileEntry *after, api_FileEntry *dest, size_t count,
                uint32_t &total) {
    total = 0;
    DIR *dir = opendir(full);
    if (!dir) return -1;
    size_t kept = 0;
    char childRel[256], childFull[256];
    for (struct dirent *e = readdir(dir); e != nullptr; e = readdir(dir)) {
        if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..") || strlen(e->d_name) >= sizeof(api_FileEntry::name))
            continue;
        if (!childPath(childRel, sizeof(childRel), rel, e->d_name) || !validPath(childRel) ||
            !childPath(childFull, sizeof(childFull), full, e->d_name))
            continue;
        ++total;
        api_FileEntry candidate;
        strcpy(candidate.name, e->d_name);
        candidate.is_directory = isDirectory(*e, childFull);
        candidate.size = 0;
        if (after && !listedBefore(*after, candidate)) continue;
        if (kept == count && !listedBefore(candidate, dest[kept - 1])) continue;
        size_t slot = kept < count ? kept++ : count - 1;
        for (; slot > 0 && listedBefore(candidate, dest[slot - 1]); --slot) dest[slot] = dest[slot - 1];
        dest[slot] = candidate;
    }
    closedir(dir);
    return (int)kept;
}

}  // namespace detail

// One page of a directory's entries: directories first, then files, each group in byte order of the name. A name
// too long for FileEntry cannot be told to the client, who could not address it, so it is neither listed nor
// counted; `config` is likewise absent from the root. `.part` files of uploads under way are listed, as they hold
// space the client may want to delete. 200 with the page and the directory's entry count; 400 a path validPath()
// refuses; 404 no such directory.
// Memory does not grow with the directory: the page itself is the only buffer, and each scan keeps just the next
// page's worth of entries after the last one passed. The price is one scan of the directory per page up to the
// requested one, which a directory of a few hundred entries on flash affords; a scan reads no entry's metadata where
// the directory tells its kind, and only the files of the requested page are asked for their size.
inline int list(const char *rel, uint32_t offset, api_FileList &out, const char *root = MOUNT_POINT) {
    constexpr size_t PAGE = sizeof(out.entries) / sizeof(out.entries[0]);
    out.entries_count = 0;
    out.total = 0;
    std::string full;
    if (!resolve(rel, full, root)) return 400;
    api_FileEntry last;
    bool passed = false;
    for (uint32_t skip = offset; skip > 0;) {
        const size_t take = std::min<uint32_t>(skip, PAGE);
        const int got = detail::scan(full.c_str(), rel, passed ? &last : nullptr, out.entries, take, out.total);
        if (got < 0) return 404;
        if ((size_t)got < take) return 200;
        last = out.entries[take - 1];
        passed = true;
        skip -= (uint32_t)take;
    }
    const int got = detail::scan(full.c_str(), rel, passed ? &last : nullptr, out.entries, PAGE, out.total);
    if (got < 0) return 404;
    out.entries_count = (pb_size_t)got;
    char childFull[256];
    for (pb_size_t i = 0; i < out.entries_count; ++i) {
        api_FileEntry &entry = out.entries[i];
        if (!entry.is_directory && detail::childPath(childFull, sizeof(childFull), full.c_str(), entry.name))
            entry.size = (uint32_t)std::max(fileSize(childFull), 0L);
    }
    return 200;
}

// 200 the directory exists, made with its parents if need be; 400 a path validPath() refuses; 409 a file is in the
// way; 500 the filesystem refused.
inline int mkdir(const char *rel, const char *root = MOUNT_POINT) {
    std::string full;
    if (!resolve(rel, full, root)) return 400;
    struct stat st;
    if (stat(full.c_str(), &st) == 0) return S_ISDIR(st.st_mode) ? 200 : 409;
    return FileSystem::mkdirRecursive(full.c_str()) ? 200 : 500;
}

}  // namespace file_transfer
