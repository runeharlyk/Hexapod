#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <ota_image.h>

// The app slot an update is written to: esp_ota_* on the robot (ota_flash.h), a fake in the host tests.
class OtaFlash {
  public:
    virtual ~OtaFlash() = default;
    // Bytes the next app slot holds; 0 when the partition table has none.
    virtual uint32_t capacity() = 0;
    virtual bool begin() = 0;
    virtual bool write(const uint8_t* data, size_t size) = 0;
    // Closes the slot and validates the image in it; releases the slot whatever it returns.
    virtual bool end() = 0;
    // Makes the validated image the one booted next.
    virtual bool activate() = 0;
    virtual void abort() = 0;
};

// The status a request is answered with, and on refusal the reason the app shows.
struct OtaReply {
    uint32_t status;
    const char* reason;
};

// One firmware update over the protobuf link at a time, owned by the client that started it. The
// image goes into the slot the robot does not run from, in order; the boot partition changes only
// when all of it arrived and validated, so an update that fails at any point leaves the robot booting
// what it runs now. `busy` is shared with the HTTP update path (ota_service.h), so the two cannot
// write the slot at once. Called from the adapter tasks and, for the timeout, from the service task.
class OtaSession {
  public:
    static constexpr uint32_t TIMEOUT_MS = 30000;
    static constexpr OtaReply OK{200, ""};

    // A client is its adapter and its id on that adapter: ids repeat across transports.
    struct Client {
        const void* adapter;
        int id;
        bool operator==(const Client& other) const { return adapter == other.adapter && id == other.id; }
    };

    OtaSession(OtaFlash& flash, std::atomic<bool>& busy) : flash_(flash), busy_(busy) {}

    OtaReply start(uint32_t size, Client client, uint32_t nowMs) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_.exchange(true)) return {409, "An update is already running"};
        const uint32_t capacity = flash_.capacity();
        if (capacity == 0) return release({507, "This partition table has no slot for updates"});
        if (size == 0 || size > capacity) return release({413, "The image does not fit the update slot"});
        if (!flash_.begin()) return release({500, "Could not open the update slot"});
        running_ = true;
        client_ = client;
        size_ = size;
        written_ = 0;
        nextChunk_ = 0;
        lastActivityMs_ = nowMs;
        return OK;
    }

    OtaReply chunk(uint32_t index, const uint8_t* data, size_t size, Client client, uint32_t nowMs) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!owns(client)) return NOT_RUNNING;
        lastActivityMs_ = nowMs;
        if (index != nextChunk_) return abort({400, "A chunk arrived out of order"});
        if (index == 0 && !ota_image::isAppImage(data, size)) {
            return abort({415, "Not an app image; a factory image goes through the web flasher"});
        }
        if (size > size_ - written_) return abort({400, "More data than announced"});
        if (!flash_.write(data, size)) return abort({500, "Writing the update slot failed"});
        written_ += size;
        nextChunk_++;
        return OK;
    }

    OtaReply finish(Client client) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!owns(client)) return NOT_RUNNING;
        if (written_ != size_) return abort({400, "The image is incomplete"});
        running_ = false;
        if (!flash_.end()) return release({422, "The robot rejected the image"});
        if (!flash_.activate()) return release({500, "Could not select the new image"});
        // The slot stays claimed: the robot restarts into the new image next.
        return OK;
    }

    // Ends an update that fell silent for longer than TIMEOUT_MS, as a client that left does.
    void expire(uint32_t nowMs) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (running_ && nowMs - lastActivityMs_ > TIMEOUT_MS) abort(OK);
    }

    bool running() const { return running_; }

  private:
    static constexpr OtaReply NOT_RUNNING{409, "No update running"};

    bool owns(Client client) const { return running_ && client == client_; }

    OtaReply release(OtaReply reply) {
        busy_ = false;
        return reply;
    }

    OtaReply abort(OtaReply reply) {
        flash_.abort();
        running_ = false;
        return release(reply);
    }

    OtaFlash& flash_;
    std::atomic<bool>& busy_;
    std::mutex mutex_;
    std::atomic<bool> running_{false};
    Client client_{nullptr, -1};
    uint32_t size_ = 0;
    uint32_t written_ = 0;
    uint32_t nextChunk_ = 0;
    uint32_t lastActivityMs_ = 0;
};
