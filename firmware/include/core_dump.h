#pragma once

// The crash the robot kept: on a panic the IDF writes an ELF core dump to the coredump partition, and these answer
// the status page's questions about it (CoreDumpGet, CoreDumpRead, CoreDumpErase in message.proto) and serve its
// bytes to GET /api/coredump. The IDF's side is core_dump_flash.h; this half runs on the host tests.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <mutex>

#include <platform_shared/message.pb.h>

namespace core_dump {

// What esp_core_dump_get_summary and esp_core_dump_get_panic_reason read from the dump, in the IDF's sizes.
struct Summary {
    char task[16];
    uint32_t pc;
    uint32_t backtrace[16];
    uint32_t depth; // entries in backtrace, the faulting PC first
    bool backtraceCorrupted;
    char panicReason[128];
    char elfSha256[65]; // CONFIG_APP_RETRIEVE_LEN_ELF_SHA hex digits, at most 64
};

// The coredump partition: the IDF's on the robot, a buffer in the host tests.
class Store {
  public:
    virtual ~Store() = default;
    // Bytes of the stored dump, its checksum included; 0 when none is stored.
    virtual uint32_t size() = 0;
    // False when the dump fails its checksum, as one cut short by a reset does, or the IDF cannot parse it.
    virtual bool summary(Summary& out) = 0;
    virtual bool read(uint32_t offset, uint8_t* into, size_t length) = 0;
    virtual bool erase() = 0;
};

// The fewest digits CONFIG_APP_RETRIEVE_LEN_ELF_SHA keeps; fewer name no build.
constexpr size_t MIN_HASH_DIGITS = 8;

// Whether the dump's hash begins the running build's: a firmware keeps only CONFIG_APP_RETRIEVE_LEN_ELF_SHA digits.
inline bool sameFirmware(const char* dumped, const char* running) {
    const size_t digits = strlen(dumped);
    return digits >= MIN_HASH_DIGITS && digits <= strlen(running) && strncmp(dumped, running, digits) == 0;
}

template <size_t N, size_t M>
inline void copyTerminated(char (&into)[N], const char (&from)[M]) {
    const size_t length = strnlen(from, std::min(N - 1, M));
    memcpy(into, from, length);
    into[length] = '\0';
}

// The format this firmware writes, in the low half of the dump header's second word (core_dump_header_t.version):
// COREDUMP_VERSION_ELF_CRC32 in the IDF's esp_core_dump_types.h, an ELF dump with a CRC32 checksum. The high half
// is the chip's ID.
constexpr uint32_t VERSION_OFFSET = 4;
constexpr uint32_t ELF_CRC32 = 0x0102;

// esp_core_dump_get_summary parses any dump as an ELF one, so a BIN dump would have its offsets followed wherever
// they point.
inline bool elfCrc32(Store& store) {
    uint32_t version = 0;
    return store.read(VERSION_OFFSET, reinterpret_cast<uint8_t*>(&version), sizeof(version)) &&
           (version & 0xFFFF) == ELF_CRC32;
}

inline void report(Store& store, const char* runningElfSha256, socket_message_CoreDumpReport& out) {
    out = socket_message_CoreDumpReport_init_zero;
    out.size = store.size();
    out.stored = out.size > 0;
    if (!out.stored) return;
    Summary crash {};
    out.readable = elfCrc32(store) && store.summary(crash);
    if (!out.readable) return;
    copyTerminated(out.task, crash.task);
    out.pc = crash.pc;
    out.backtrace_count = std::min<uint32_t>({crash.depth, std::size(crash.backtrace), std::size(out.backtrace)});
    std::copy_n(crash.backtrace, out.backtrace_count, out.backtrace);
    out.backtrace_corrupted = crash.backtraceCorrupted;
    copyTerminated(out.panic_reason, crash.panicReason);
    copyTerminated(out.app_elf_sha256, crash.elfSha256);
    out.elf_matches_running = sameFirmware(out.app_elf_sha256, runningElfSha256);
}

// As file_transfer::read answers a FileReadChunk: 200 with the bytes at offset (none past the end) and the dump's
// size; 404 no dump; 400 a length over the reply's room; 500 the flash read failed.
inline uint32_t readChunk(Store& store, uint32_t offset, uint32_t length, api_FileChunk& chunk) {
    chunk.content.size = 0;
    if (length > sizeof(chunk.content.bytes)) return 400;
    const uint32_t size = store.size();
    if (size == 0) return 404;
    chunk.total_size = size;
    const uint32_t n = offset < size ? std::min(length, size - offset) : 0;
    if (n > 0 && !store.read(offset, chunk.content.bytes, n)) return 500;
    chunk.content.size = n;
    return 200;
}

// The report on the stored dump, made by one check of it. A check reads the whole dump from flash, and every flash
// read parks the other core, the control task's, so app_main checks before that task starts and the status page is
// answered from the report. The dump changes only by a crash, which restarts the robot, or by an erase, which checks
// it again at rest.
class Readout {
  public:
    explicit Readout(Store& store) : store_(store) {}

    void check(const char* runningElfSha256) {
        socket_message_CoreDumpReport checked;
        core_dump::report(store_, runningElfSha256, checked);
        std::lock_guard<std::mutex> lock(mutex_);
        report_ = checked;
    }

    void report(socket_message_CoreDumpReport& out) {
        std::lock_guard<std::mutex> lock(mutex_);
        out = report_;
    }

    // 409 unless the robot rests: erasing flash stalls both cores, the control loop's too. 500 the erase failed. The
    // report is the one after: the check's when refused, a new check's otherwise.
    uint32_t erase(bool resting, const char* runningElfSha256, socket_message_CoreDumpReport& out) {
        if (!resting) {
            report(out);
            return 409;
        }
        const uint32_t status = store_.erase() ? 200 : 500;
        check(runningElfSha256);
        report(out);
        return status;
    }

  private:
    Store& store_;
    std::mutex mutex_;
    socket_message_CoreDumpReport report_ = socket_message_CoreDumpReport_init_zero;
};

} // namespace core_dump
