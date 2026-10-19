#pragma once

// The robot's server-to-client topics by tag, as platform_shared/topics.json lists them (test/test_topic_snapshot
// checks this table against it). A state topic holds one current value and every change of it is pushed; a stream is a
// sequence of frames. A tag not listed is no topic.

#include <cstdint>
#include <platform_shared/message.pb.h>

enum class TopicKind { STATE, STREAM };

struct TopicKindEntry {
    int32_t tag;
    TopicKind kind;
};

inline constexpr TopicKindEntry TOPIC_KINDS[] = {
    {socket_message_Message_imu_tag, TopicKind::STREAM},
    {socket_message_Message_mode_tag, TopicKind::STATE},
    {socket_message_Message_analytics_tag, TopicKind::STREAM},
    {socket_message_Message_gait_tag, TopicKind::STATE},
    {socket_message_Message_angles_tag, TopicKind::STREAM},
    {socket_message_Message_wifi_status_tag, TopicKind::STATE},
    {socket_message_Message_ap_status_tag, TopicKind::STATE},
    {socket_message_Message_ota_status_tag, TopicKind::STATE},
    {socket_message_Message_animation_status_tag, TopicKind::STATE},
    {socket_message_Message_telemetry_batch_tag, TopicKind::STREAM},
    {socket_message_Message_goal_status_tag, TopicKind::STATE},
};
