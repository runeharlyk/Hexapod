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
#include <communication/topics.h>

class CommAdapterBase;

// Where a state topic's snapshot puts its owner's current value (communication/topics.h). Only a subscribe makes one,
// with the adapter's encode lock held, so a snapshot sends nowhere else and at no other time.
class SnapshotSink {
  public:
    template <typename T>
    void put(const T& value);

  private:
    friend class CommAdapterBase;
    SnapshotSink(CommAdapterBase& adapter, int cid) : adapter_(adapter), cid_(cid) {}
    SnapshotSink(const SnapshotSink&) = delete;
    SnapshotSink& operator=(const SnapshotSink&) = delete;

    CommAdapterBase& adapter_;
    int cid_;
    bool delivered_ = false;
};

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

    // The topics this adapter serves; a tag not in the table is served as a stream.
    template <size_t N>
    void setTopics(const Topic (&topics)[N]) {
        topics_ = topics;
        topicCount_ = N;
    }

    // Fired when a tag gains its first subscriber or loses its last.
    void onSubscriptionChange(std::function<void(int32_t)> cb) { tagChangeCb_ = std::move(cb); }

    // Whether a client is connected on this transport, whatever it subscribes to.
    virtual bool hasClient() const = 0;

    // Fired after a client has gone and its subscriptions are dropped, outside every adapter lock, with this adapter
    // and the client's id: a request is owned by both, since ids repeat across transports.
    void onClientGone(std::function<void(const CommAdapterBase*, int)> cb) { clientGoneCb_ = std::move(cb); }

    // Fired for every frame a client sends, on the transport's task, with this adapter and the client's id.
    void onClientHeard(std::function<void(const CommAdapterBase*, int)> cb) { clientHeardCb_ = std::move(cb); }

    template <typename T>
    void on(std::function<void(const T&, int)> handler) {
        decoder_.on<T>(handler);
    }

    // Reached from the event bus worker, the service loop and the transport handler tasks, so the shared message
    // struct and encode buffer are held under encodeMutex_ for the whole encode-and-send. Keeping them shared
    // instead of stack-local matters: socket_message_Message is over a kilobyte.
    template <typename T>
    void emit(const T& data, int clientId = -1) {
        if (clientId < 0 && !hasSubscribers(MessageTraits<T>::tag)) return;
        ScopedLock lock(encodeMutex_);
        encodeAndSend(data, clientId);
    }

    // From the service loop: a state push a client could not take marked its subscription dirty (sendToSubscribers),
    // and each dirty client that can take a frame now gets the topic's current value instead, once. A client that still
    // cannot keeps its mark for the next pass, so the service loop never waits on one.
    void resendDirty() {
        std::vector<std::pair<int32_t, int>> due;
        {
            ScopedLock lock(mutex_);
            for (const auto& [tag, clients] : client_subscriptions_)
                for (const Subscriber& client : clients)
                    if (client.dirty) due.emplace_back(tag, client.cid);
        }
        if (due.empty()) return;
        ScopedLock lock(encodeMutex_);
        for (const auto& [tag, cid] : due) {
            if (!ready(cid) || !clearDirty(tag, cid)) continue;
            SnapshotSink sink(*this, cid);
            topicOf(tag)->snapshot(sink);
            if (!sink.delivered_) markDirty(tag, cid, true);
        }
    }

    // A directed message from a task that must not wait on one client (the service loop): sent only when the client can
    // take a frame now, and dropped otherwise, as a subscription's frame is. False when it was dropped.
    template <typename T>
    bool offer(const T& data, int clientId) {
        if (!ready(clientId)) return false;
        emit(data, clientId);
        return true;
    }

  protected:
    // Whether the frame went out whole: false when the transport dropped it, as a BLE notify without an mbuf or a
    // serial write cut short does, which a state push then marks for resendDirty.
    virtual bool send(const uint8_t* data, size_t len, int cid = -1) = 0;

    // Whether a client can take a frame now without blocking. Subscriptions stream the latest value,
    // so a frame a client cannot take is dropped rather than queued or waited on.
    virtual bool ready(int cid) { return true; }

    // The callbacks below fire outside mutex_: a handler asks every adapter whether the tag still
    // has listeners, which re-enters this lock.
    // A client resubscribes on every reconnect and fail-over, so a subscribe it already holds adds nothing; it is
    // answered again all the same. A state topic is answered with its snapshot, then every subscribe with its ack
    // (communication/topics.h). The bridge attaches before the owner is read, so a change after the read is pushed. The
    // read, the snapshot and the ack go out under encodeMutex_, which every push to this adapter takes, so a push of a
    // change made after the read follows the snapshot rather than overtaking it; a change made before the read may be
    // pushed after it too, which only repeats the value. A snapshot must therefore never wait on an EventBus worker,
    // and an owner never pushes while it holds a lock its snapshot takes (docs/firmware.md, Transports).
    void subscribe(int32_t tag, int cid = 0) {
        bool first;
        {
            ScopedLock lock(mutex_);
            auto& clients = client_subscriptions_[tag];
            first = clients.empty();
            if (!findClient(clients, cid)) clients.push_back({cid, false});
        }
        if (first) notifyTagChange(tag);
        ESP_LOGI("ProtoComm", "Client %d subscribed to tag %d", cid, (int)tag);

        ScopedLock lock(encodeMutex_);
        const Topic* topic = topicOf(tag);
        if (topic && topic->kind == TopicKind::STATE) {
            SnapshotSink sink(*this, cid);
            topic->snapshot(sink);
        }
        msg_.which_message = socket_message_Message_sub_notif_tag;
        msg_.message.sub_notif.tag = tag;
        sendSmall(cid);
    }

    void unsubscribe(int32_t tag, int cid = 0) {
        bool last;
        {
            ScopedLock lock(mutex_);
            auto& clients = client_subscriptions_[tag];
            clients.remove_if([cid](const Subscriber& client) { return client.cid == cid; });
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
                clients.remove_if([cid](const Subscriber& client) { return client.cid == cid; });
                if (clients.empty()) emptied.push_back(tag);
            }
        }
        for (int32_t tag : emptied) notifyTagChange(tag);
        if (clientGoneCb_) clientGoneCb_(this, cid);
    }

    void handleIncoming(const uint8_t* data, size_t len, int cid) {
        if (clientHeardCb_) clientHeardCb_(this, cid);
        if (!decoder_.decode(data, len, cid)) {
            ESP_LOGE("ProtoComm", "Failed to decode incoming message from client %d", cid);
        }
    }

    void sendPong(int cid) {
        ScopedLock lock(encodeMutex_);
        msg_.which_message = socket_message_Message_pongmsg_tag;
        msg_.message.pongmsg = socket_message_PongMsg_init_zero;
        sendSmall(cid);
    }

  private:
    friend class SnapshotSink;

    // Under encodeMutex_: the shared message and encode buffer. Whether a directed message went out.
    template <typename T>
    bool encodeAndSend(const T& data, int clientId) {
        constexpr pb_size_t tag = MessageTraits<T>::tag;
        msg_.which_message = tag;
        MessageTraits<T>::assign(msg_, data);

        size_t out_size = 0;
        if (!pb_get_encoded_size(&out_size, socket_message_Message_fields, &msg_)) {
            ESP_LOGE("ProtoComm", "Failed to size message (tag %d)", (int)tag);
            return false;
        }

        EncodeBuffer buffer(enc_buf_, sizeof(enc_buf_), out_size);
        if (!buffer.data()) {
            ESP_LOGE("ProtoComm", "Out of memory for %u byte message (tag %d)", (unsigned)out_size, (int)tag);
            return false;
        }

        pb_ostream_t stream = pb_ostream_from_buffer(buffer.data(), out_size);
        if (!pb_encode(&stream, socket_message_Message_fields, &msg_)) {
            ESP_LOGE("ProtoComm", "Failed to encode message (tag %d): %s", (int)tag, PB_GET_ERROR(&stream));
            return false;
        }

        if (clientId >= 0) return send(buffer.data(), stream.bytes_written, clientId);
        sendToSubscribers(tag, buffer.data(), stream.bytes_written);
        return true;
    }

    // Under encodeMutex_: msg_ as set, a pong or an ack, which fits a few bytes on the stack.
    void sendSmall(int cid) {
        uint8_t bytes[16];
        pb_ostream_t stream = pb_ostream_from_buffer(bytes, sizeof(bytes));
        if (pb_encode(&stream, socket_message_Message_fields, &msg_)) send(bytes, stream.bytes_written, cid);
    }

    const Topic* topicOf(int32_t tag) const {
        for (size_t i = 0; i < topicCount_; i++)
            if (topics_[i].tag == tag) return &topics_[i];
        return nullptr;
    }

    class ScopedLock {
      public:
        explicit ScopedLock(SemaphoreHandle_t mutex) : mutex_(mutex) { xSemaphoreTake(mutex_, portMAX_DELAY); }
        ~ScopedLock() { xSemaphoreGive(mutex_); }
        ScopedLock(const ScopedLock&) = delete;
        ScopedLock& operator=(const ScopedLock&) = delete;

      private:
        SemaphoreHandle_t mutex_;
    };

    // Falls back to the heap for a message that outgrows the static buffer.
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

    // A client on a tag's list, and whether the latest state push to it was dropped (resendDirty).
    struct Subscriber {
        int cid;
        bool dirty;
    };

    static Subscriber* findClient(std::list<Subscriber>& clients, int cid) {
        for (Subscriber& client : clients)
            if (client.cid == cid) return &client;
        return nullptr;
    }

    // Sends to a copy of the list, so a slow client never holds up subscribing or closing a session. A state push a
    // client cannot take, or whose send fails, marks it dirty for resendDirty, and one it takes clears the mark; a
    // stream frame is just lost.
    void sendToSubscribers(int32_t tag, const uint8_t* data, size_t len) {
        std::vector<Subscriber> clients;
        {
            ScopedLock lock(mutex_);
            auto it = client_subscriptions_.find(tag);
            if (it == client_subscriptions_.end()) return;
            clients.assign(it->second.begin(), it->second.end());
        }
        const Topic* topic = topicOf(tag);
        const bool state = topic && topic->kind == TopicKind::STATE;
        for (const Subscriber& client : clients) {
            const bool delivered = ready(client.cid) && send(data, len, client.cid);
            if (state && delivered == client.dirty) markDirty(tag, client.cid, !delivered);
        }
    }

    void markDirty(int32_t tag, int cid, bool dirty) {
        ScopedLock lock(mutex_);
        auto it = client_subscriptions_.find(tag);
        if (it == client_subscriptions_.end()) return;
        if (Subscriber* client = findClient(it->second, cid)) client->dirty = dirty;
    }

    // Whether the client was marked; it no longer is.
    bool clearDirty(int32_t tag, int cid) {
        ScopedLock lock(mutex_);
        auto it = client_subscriptions_.find(tag);
        Subscriber* client = it == client_subscriptions_.end() ? nullptr : findClient(it->second, cid);
        if (!client || !client->dirty) return false;
        client->dirty = false;
        return true;
    }

    SemaphoreHandle_t mutex_;
    SemaphoreHandle_t encodeMutex_;
    std::map<int32_t, std::list<Subscriber>> client_subscriptions_;
    std::function<void(int32_t)> tagChangeCb_;
    std::function<void(const CommAdapterBase*, int)> clientGoneCb_;
    std::function<void(const CommAdapterBase*, int)> clientHeardCb_;

    void notifyTagChange(int32_t tag) {
        if (tagChangeCb_) tagChangeCb_(tag);
    }
    ProtoDecoder decoder_;
    socket_message_Message msg_ = socket_message_Message_init_zero;
    uint8_t enc_buf_[PROTO_BUFFER_SIZE];
    const Topic* topics_ = nullptr;
    size_t topicCount_ = 0;
};

template <typename T>
void SnapshotSink::put(const T& value) {
    delivered_ = adapter_.encodeAndSend(value, cid_);
}
