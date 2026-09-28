#pragma once

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdlib>
#include <functional>
#include <list>
#include <map>
#include <vector>
#include <type_traits>
#include <communication/proto_helpers.h>

class CommAdapterBase {
  public:
    CommAdapterBase() {
        mutex_ = xSemaphoreCreateMutex();
        encodeMutex_ = xSemaphoreCreateMutex();
        decoder_.onSubscribe([this](int32_t tag, int cid) { subscribe(tag, cid); });
        decoder_.onUnsubscribe([this](int32_t tag, int cid) { unsubscribe(tag, cid); });
        decoder_.onPing([this](int cid) { sendPong(cid); });
    }
    ~CommAdapterBase() {
        vSemaphoreDelete(mutex_);
        vSemaphoreDelete(encodeMutex_);
    }

    virtual void begin() {}

    bool hasSubscribers(int32_t tag) {
        ScopedLock lock(mutex_);
        auto it = client_subscriptions_.find(tag);
        return it != client_subscriptions_.end() && !it->second.empty();
    }

    ProtoDecoder& decoder() { return decoder_; }

    // Fired when a tag gains its first subscriber or loses its last.
    void onSubscriptionChange(std::function<void(int32_t)> cb) { tagChangeCb_ = std::move(cb); }

    template <typename T>
    void on(std::function<void(const T&, int)> handler) {
        decoder_.on<T>(handler);
    }

    // Reached from the event bus worker, the service loop and the transport handler tasks, so the shared message
    // struct and encode buffer are held under encodeMutex_ for the whole encode-and-send. Keeping them shared
    // instead of stack-local matters: socket_message_Message is over a kilobyte.
    template <typename T>
    void emit(const T& data, int clientId = -1) {
        constexpr pb_size_t tag = MessageTraits<T>::tag;

        if (clientId < 0 && !hasSubscribers(tag)) return;

        ScopedLock lock(encodeMutex_);

        msg_.which_message = tag;
        MessageTraits<T>::assign(msg_, data);

        size_t out_size = 0;
        if (!pb_get_encoded_size(&out_size, socket_message_Message_fields, &msg_)) {
            ESP_LOGE("ProtoComm", "Failed to size message (tag %d)", (int)tag);
            return;
        }

        EncodeBuffer buffer(enc_buf_, sizeof(enc_buf_), out_size);
        if (!buffer.data()) {
            ESP_LOGE("ProtoComm", "Out of memory for %u byte message (tag %d)", (unsigned)out_size, (int)tag);
            return;
        }

        pb_ostream_t stream = pb_ostream_from_buffer(buffer.data(), out_size);
        if (!pb_encode(&stream, socket_message_Message_fields, &msg_)) {
            ESP_LOGE("ProtoComm", "Failed to encode message (tag %d): %s", (int)tag, PB_GET_ERROR(&stream));
            return;
        }

        if (clientId >= 0) {
            send(buffer.data(), stream.bytes_written, clientId);
        } else {
            sendToSubscribers(tag, buffer.data(), stream.bytes_written);
        }
    }

  protected:
    virtual void send(const uint8_t* data, size_t len, int cid = -1) = 0;

    // The callbacks below fire outside mutex_: a handler asks every adapter whether the tag still
    // has listeners, which re-enters this lock.
    void subscribe(int32_t tag, int cid = 0) {
        bool first;
        {
            ScopedLock lock(mutex_);
            auto& clients = client_subscriptions_[tag];
            first = clients.empty();
            clients.push_back(cid);
        }
        if (first) notifyTagChange(tag);
        ESP_LOGI("ProtoComm", "Client %d subscribed to tag %d", cid, (int)tag);
    }

    void unsubscribe(int32_t tag, int cid = 0) {
        bool last;
        {
            ScopedLock lock(mutex_);
            auto& clients = client_subscriptions_[tag];
            clients.remove(cid);
            last = clients.empty();
        }
        if (last) notifyTagChange(tag);
        ESP_LOGI("ProtoComm", "Client %d unsubscribed from tag %d", cid, (int)tag);
    }

    void removeClient(int cid) {
        std::vector<int32_t> emptied;
        {
            ScopedLock lock(mutex_);
            for (auto& [tag, clients] : client_subscriptions_) {
                if (clients.empty()) continue;
                clients.remove(cid);
                if (clients.empty()) emptied.push_back(tag);
            }
        }
        for (int32_t tag : emptied) notifyTagChange(tag);
    }

    void handleIncoming(const uint8_t* data, size_t len, int cid) {
        if (!decoder_.decode(data, len, cid)) {
            ESP_LOGE("ProtoComm", "Failed to decode incoming message from client %d", cid);
        }
    }

    void sendPong(int cid) {
        uint8_t pongBuffer[16];
        ScopedLock lock(encodeMutex_);
        msg_.which_message = socket_message_Message_pongmsg_tag;
        msg_.message.pongmsg = socket_message_PongMsg_init_zero;
        pb_ostream_t stream = pb_ostream_from_buffer(pongBuffer, sizeof(pongBuffer));
        if (pb_encode(&stream, socket_message_Message_fields, &msg_)) {
            send(pongBuffer, stream.bytes_written, cid);
        }
    }

  private:
    class ScopedLock {
      public:
        explicit ScopedLock(SemaphoreHandle_t mutex) : mutex_(mutex) { xSemaphoreTake(mutex_, portMAX_DELAY); }
        ~ScopedLock() { xSemaphoreGive(mutex_); }
        ScopedLock(const ScopedLock&) = delete;
        ScopedLock& operator=(const ScopedLock&) = delete;

      private:
        SemaphoreHandle_t mutex_;
    };

    // Falls back to the heap for the few messages (I2C scan, WiFi network list) that outgrow the static buffer.
    class EncodeBuffer {
      public:
        EncodeBuffer(uint8_t* staticBuffer, size_t staticSize, size_t needed) {
            if (needed <= staticSize) {
                data_ = staticBuffer;
            } else {
                heap_ = (uint8_t*)malloc(needed);
                data_ = heap_;
            }
        }
        ~EncodeBuffer() { free(heap_); }
        EncodeBuffer(const EncodeBuffer&) = delete;
        EncodeBuffer& operator=(const EncodeBuffer&) = delete;

        uint8_t* data() const { return data_; }

      private:
        uint8_t* heap_ = nullptr;
        uint8_t* data_ = nullptr;
    };

    void sendToSubscribers(int32_t tag, const uint8_t* data, size_t len) {
        ScopedLock lock(mutex_);
        auto it = client_subscriptions_.find(tag);
        if (it == client_subscriptions_.end()) return;
        for (int cid : it->second) {
            send(data, len, cid);
        }
    }

    SemaphoreHandle_t mutex_;
    SemaphoreHandle_t encodeMutex_;
    std::map<int32_t, std::list<int>> client_subscriptions_;
    std::function<void(int32_t)> tagChangeCb_;

    void notifyTagChange(int32_t tag) {
        if (tagChangeCb_) tagChangeCb_(tag);
    }
    ProtoDecoder decoder_;
    socket_message_Message msg_ = socket_message_Message_init_zero;
    uint8_t enc_buf_[PROTO_BUFFER_SIZE];
};
