#pragma once

#include <pb_encode.h>
#include <pb_decode.h>
#include <platform_shared/message.pb.h>
#include <functional>
#include <map>

#define PROTO_BUFFER_SIZE 2048

template <typename T>
struct MessageTraits;

#define DEFINE_MESSAGE_TRAITS(DataType, field)                                                   \
    template <>                                                                                  \
    struct MessageTraits<socket_message_##DataType> {                                            \
        static constexpr pb_size_t tag = socket_message_Message_##field##_tag;                   \
        static void assign(socket_message_Message& msg, const socket_message_##DataType& data) { \
            msg.message.field = data;                                                            \
        }                                                                                        \
        static const socket_message_##DataType& access(const socket_message_Message& msg) {      \
            return msg.message.field;                                                            \
        }                                                                                        \
    };

DEFINE_MESSAGE_TRAITS(ControllerInputData, input)
DEFINE_MESSAGE_TRAITS(ModeData, mode)
DEFINE_MESSAGE_TRAITS(GaitData, gait)
DEFINE_MESSAGE_TRAITS(AnglesData, angles)
DEFINE_MESSAGE_TRAITS(IMUData, imu)
DEFINE_MESSAGE_TRAITS(ServoPWMData, servo_pwm)
DEFINE_MESSAGE_TRAITS(ServoStateData, servo_state)
DEFINE_MESSAGE_TRAITS(JointAngleData, joint_angle)
DEFINE_MESSAGE_TRAITS(AnalyticsData, analytics)
DEFINE_MESSAGE_TRAITS(CorrelationRequest, correlation_request)
DEFINE_MESSAGE_TRAITS(CorrelationResponse, correlation_response)
DEFINE_MESSAGE_TRAITS(SystemCommandData, system_command)
DEFINE_MESSAGE_TRAITS(OtaStatusData, ota_status)
DEFINE_MESSAGE_TRAITS(TelemetryBatch, telemetry_batch)
DEFINE_MESSAGE_TRAITS(AnimationPlay, animation_play)
DEFINE_MESSAGE_TRAITS(AnimationStop, animation_stop)
DEFINE_MESSAGE_TRAITS(PoseData, pose)
DEFINE_MESSAGE_TRAITS(AnimationStatus, animation_status)
DEFINE_MESSAGE_TRAITS(GoalStatus, goal_status)

#undef DEFINE_MESSAGE_TRAITS

// api-package types in the oneof need a hand-written trait (the macro is socket_message-only).
template <>
struct MessageTraits<api_WifiStatus> {
    static constexpr pb_size_t tag = socket_message_Message_wifi_status_tag;
    static void assign(socket_message_Message& msg, const api_WifiStatus& data) { msg.message.wifi_status = data; }
    static const api_WifiStatus& access(const socket_message_Message& msg) { return msg.message.wifi_status; }
};

template <>
struct MessageTraits<api_APStatus> {
    static constexpr pb_size_t tag = socket_message_Message_ap_status_tag;
    static void assign(socket_message_Message& msg, const api_APStatus& data) { msg.message.ap_status = data; }
    static const api_APStatus& access(const socket_message_Message& msg) { return msg.message.ap_status; }
};

template <>
struct MessageTraits<api_ServoSettings> {
    static constexpr pb_size_t tag = socket_message_Message_servo_settings_tag;
    static void assign(socket_message_Message& msg, const api_ServoSettings& data) {
        msg.message.servo_settings = data;
    }
    static const api_ServoSettings& access(const socket_message_Message& msg) { return msg.message.servo_settings; }
};

template <>
struct MessageTraits<api_PeripheralSettings> {
    static constexpr pb_size_t tag = socket_message_Message_peripheral_settings_tag;
    static void assign(socket_message_Message& msg, const api_PeripheralSettings& data) {
        msg.message.peripheral_settings = data;
    }
    static const api_PeripheralSettings& access(const socket_message_Message& msg) {
        return msg.message.peripheral_settings;
    }
};

template <>
struct MessageTraits<api_WifiSettings> {
    static constexpr pb_size_t tag = socket_message_Message_wifi_settings_tag;
    static void assign(socket_message_Message& msg, const api_WifiSettings& data) { msg.message.wifi_settings = data; }
    static const api_WifiSettings& access(const socket_message_Message& msg) { return msg.message.wifi_settings; }
};

template <>
struct MessageTraits<api_APSettings> {
    static constexpr pb_size_t tag = socket_message_Message_ap_settings_tag;
    static void assign(socket_message_Message& msg, const api_APSettings& data) { msg.message.ap_settings = data; }
    static const api_APSettings& access(const socket_message_Message& msg) { return msg.message.ap_settings; }
};

template <>
struct MessageTraits<api_MDNSSettings> {
    static constexpr pb_size_t tag = socket_message_Message_mdns_settings_tag;
    static void assign(socket_message_Message& msg, const api_MDNSSettings& data) { msg.message.mdns_settings = data; }
    static const api_MDNSSettings& access(const socket_message_Message& msg) { return msg.message.mdns_settings; }
};

class ProtoDecoder {
  public:
    using SubscribeHandler = std::function<void(int32_t tag, int clientId)>;
    using UnsubscribeHandler = std::function<void(int32_t tag, int clientId)>;
    using PingHandler = std::function<void(int clientId)>;

    void onSubscribe(SubscribeHandler handler) { subscribeHandler_ = handler; }
    void onUnsubscribe(UnsubscribeHandler handler) { unsubscribeHandler_ = handler; }
    void onPing(PingHandler handler) { pingHandler_ = handler; }

    template <typename T>
    void on(std::function<void(const T&, int)> handler) {
        handlers_[MessageTraits<T>::tag] = [handler, this](int clientId) {
            handler(MessageTraits<T>::access(msg_), clientId);
        };
    }

    bool decode(const uint8_t* data, size_t len, int clientId) {
        pb_istream_t stream = pb_istream_from_buffer(data, len);

        if (!pb_decode(&stream, socket_message_Message_fields, &msg_)) {
            return false;
        }

        switch (msg_.which_message) {
            case socket_message_Message_sub_notif_tag:
                if (subscribeHandler_) subscribeHandler_(msg_.message.sub_notif.tag, clientId);
                return true;

            case socket_message_Message_unsub_notif_tag:
                if (unsubscribeHandler_) unsubscribeHandler_(msg_.message.unsub_notif.tag, clientId);
                return true;

            case socket_message_Message_pingmsg_tag:
                if (pingHandler_) pingHandler_(clientId);
                return true;

            default: {
                auto it = handlers_.find(msg_.which_message);
                if (it != handlers_.end()) {
                    it->second(clientId);
                    return true;
                }
                return false;
            }
        }
    }

  private:
    socket_message_Message msg_ = socket_message_Message_init_zero;
    SubscribeHandler subscribeHandler_;
    UnsubscribeHandler unsubscribeHandler_;
    PingHandler pingHandler_;
    std::map<pb_size_t, std::function<void(int)>> handlers_;
};
