#pragma once

// The coredump partition the panic handler writes, read and erased through the IDF's esp_core_dump_* calls
// (components/espcoredump/include/esp_core_dump.h in IDF 5.4), and GET /api/coredump, which streams it.

#include <esp_core_dump.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_partition.h>

#include <core_dump.h>
#include <communication/webserver.h>

#if !CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH || !CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF || !CONFIG_ESP_COREDUMP_CHECKSUM_CRC32
#error "the core-dump readout reads ELF dumps with a CRC32 checksum from flash (core_dump::ELF_CRC32)"
#endif
#if !CONFIG_IDF_TARGET_ARCH_XTENSA
#error "the backtrace is read from the Xtensa summary; a RISC-V dump holds the stack instead"
#endif

class EspCoreDumpStore : public core_dump::Store {
  public:
    uint32_t size() override {
        size_t address = 0, size = 0;
        return esp_core_dump_image_get(&address, &size) == ESP_OK ? size : 0;
    }

    // esp_core_dump_get_summary parses the mapped dump without checking it, so a dump cut short by a reset is
    // checked first, which reads the whole dump in 32-byte steps; core_dump::Readout has that done once, at boot. The
    // summary starts zeroed: the IDF fills only what the dump's notes and segments hold.
    bool summary(core_dump::Summary& out) override {
        if (esp_core_dump_image_check() != ESP_OK) return false;
        esp_core_dump_summary_t summary {};
        if (esp_core_dump_get_summary(&summary) != ESP_OK) return false;
        core_dump::copyTerminated(out.task, summary.exc_task);
        out.pc = summary.exc_pc;
        // The IDF writes the faulting PC and up to 16 frames after it, so a deep stack's 17th entry lands on depth,
        // which then reads 17 (esp_core_dump_summary_parse_backtrace_info, port/xtensa/core_dump_port.c).
        out.depth = std::min<uint32_t>(summary.exc_bt_info.depth, std::size(out.backtrace));
        std::copy_n(summary.exc_bt_info.bt, out.depth, out.backtrace);
        out.backtraceCorrupted = summary.exc_bt_info.corrupted;
        static_assert(sizeof(summary.app_elf_sha256) <= sizeof(out.elfSha256));
        core_dump::copyTerminated(
            out.elfSha256, reinterpret_cast<const char(&)[sizeof(summary.app_elf_sha256)]>(summary.app_elf_sha256));
        // Only a task-watchdog panic leaves a reason; an assert's or abort's message goes to the serial log alone.
        if (esp_core_dump_get_panic_reason(out.panicReason, sizeof(out.panicReason)) != ESP_OK)
            out.panicReason[0] = '\0';
        return true;
    }

    // The dump starts at the partition's first byte, header and all, as esp-coredump's raw format reads it.
    bool read(uint32_t offset, uint8_t* into, size_t length) override {
        const esp_partition_t* partition =
            esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr);
        return partition && esp_partition_read(partition, offset, into, length) == ESP_OK;
    }

    bool erase() override {
        const esp_err_t err = esp_core_dump_image_erase();
        if (err != ESP_OK) ESP_LOGW("core_dump", "Erase failed: %s", esp_err_to_name(err));
        return err == ESP_OK;
    }
};

namespace core_dump {

// GET /api/coredump, on the HTTP server's task: the stored dump in 512-byte chunks from flash, for
// `esp-coredump info_corefile -t raw`. 404 when none is stored. A read that fails partway drops the connection, so
// the transfer ends without its last chunk and the client sees it cut short.
inline esp_err_t serveDownload(httpd_req_t* req, Store& store) {
    api_FileChunk chunk;
    uint32_t offset = 0;
    uint32_t status = readChunk(store, offset, sizeof(chunk.content.bytes), chunk);
    if (status == 404) return WebServer::sendError(req, 404, "No core dump stored");
    if (status != 200) return WebServer::sendError(req, 500, "The core dump could not be read");
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"coredump.bin\"");
    while (status == 200 && chunk.content.size > 0) {
        if (httpd_resp_send_chunk(req, reinterpret_cast<const char*>(chunk.content.bytes), chunk.content.size) !=
            ESP_OK)
            return ESP_FAIL;
        offset += chunk.content.size;
        status = readChunk(store, offset, sizeof(chunk.content.bytes), chunk);
    }
    return status == 200 ? httpd_resp_send_chunk(req, nullptr, 0) : ESP_FAIL;
}

} // namespace core_dump
