#pragma once

// A CommAdapterBase on a fake transport that decodes what it sends, for the host tests of subscriptions and state
// topics (test_topic_snapshot, test_settings_topics).

#include <unity.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <pb_decode.h>
#include <pb_encode.h>
#include <communication/comm_base.hpp>

struct Frame {
    int cid;
    socket_message_Message message;
};

// A transport whose clients can take a frame while `writable`; it keeps every frame it sent, decoded, with its client.
// While `sendsFail`, a client looks ready but every send fails, as a BLE notify does without an mbuf, and so do the
// next `failNext` sends. The `slowClient` cannot take a frame, and a send to it blocks until `releaseSlow()`, as
// httpd's send to a full socket does for up to its 5 s send timeout.
class FakeAdapter final : public CommAdapterBase {
  public:
    template <size_t N>
    explicit FakeAdapter(const Topic (&topics)[N]) {
        setTopics(topics);
    }

    std::atomic<bool> writable {true};
    std::atomic<bool> sendsFail {false};
    std::atomic<int> failNext {0};
    std::atomic<int> slowClient {-1};
    std::atomic<bool> blockedOnSlow {false};

    void releaseSlow() {
        {
            std::lock_guard<std::mutex> lock(slowMutex_);
            slowReleased_ = true;
        }
        slowReleasedCv_.notify_all();
    }
    bool hasClient() const override { return true; }

    void receive(const socket_message_Message& message, int cid) {
        size_t size = 0;
        TEST_ASSERT_TRUE(pb_get_encoded_size(&size, socket_message_Message_fields, &message));
        std::vector<uint8_t> bytes(size);
        pb_ostream_t stream = pb_ostream_from_buffer(bytes.data(), bytes.size());
        TEST_ASSERT_TRUE(pb_encode(&stream, socket_message_Message_fields, &message));
        handleIncoming(bytes.data(), stream.bytes_written, cid);
    }

    void subscribeTo(int32_t tag, int cid) {
        socket_message_Message message = socket_message_Message_init_zero;
        message.which_message = socket_message_Message_sub_notif_tag;
        message.message.sub_notif.tag = tag;
        receive(message, cid);
    }

    std::vector<Frame> to(int cid) {
        std::lock_guard<std::mutex> lock(sentMutex_);
        std::vector<Frame> frames;
        for (const Frame& frame : sent_)
            if (frame.cid == cid) frames.push_back(frame);
        return frames;
    }

    size_t sentCount() {
        std::lock_guard<std::mutex> lock(sentMutex_);
        return sent_.size();
    }

    void forget() {
        std::lock_guard<std::mutex> lock(sentMutex_);
        sent_.clear();
    }

  protected:
    bool ready(int cid) override { return writable && cid != slowClient; }
    bool send(const uint8_t* data, size_t len, int cid) override {
        if (cid == slowClient) {
            blockedOnSlow = true;
            std::unique_lock<std::mutex> lock(slowMutex_);
            slowReleasedCv_.wait(lock, [this] { return slowReleased_; });
        }
        if (sendsFail) return false;
        if (failNext > 0) {
            failNext--;
            return false;
        }
        Frame frame {cid, socket_message_Message_init_zero};
        pb_istream_t stream = pb_istream_from_buffer(data, len);
        TEST_ASSERT_TRUE(pb_decode(&stream, socket_message_Message_fields, &frame.message));
        std::lock_guard<std::mutex> lock(sentMutex_);
        sent_.push_back(frame);
        return true;
    }

  private:
    std::mutex sentMutex_;
    std::vector<Frame> sent_;
    std::mutex slowMutex_;
    std::condition_variable slowReleasedCv_;
    bool slowReleased_ = false;
};
