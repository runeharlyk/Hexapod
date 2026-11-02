// AnimationRunner with the clips this test writes: its validation refuses a clip with a frame that brings two legs
// within LEG_CLEARANCE_MM, naming the frame and the pair as simulation/src/robot/animation.py's clearance_error does
// (test_animation.py holds the Python side to the same words for the same clip), and its player holds such a frame
// apart, busy and reporting it until the angles have caught up.

// The store reads the clips this test writes into a directory of its own under the build directory.
#define ANIMATION_DIRECTORY ".pio/test_animation_runner"

#include <unity.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <pb_encode.h>
#include <animation/animation_runner.h>

namespace {

constexpr float STANCE[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

// The clip as the store reads it, `<name>.pb` in ANIMATION_DIRECTORY.
void writeAnimation(const animation_Animation& a) {
    uint8_t bytes[animation_Animation_size];
    pb_ostream_t stream = pb_ostream_from_buffer(bytes, sizeof(bytes));
    TEST_ASSERT_TRUE(pb_encode(&stream, animation_Animation_fields, &a));
    std::filesystem::create_directories(ANIMATION_DIRECTORY);
    FILE* f = fopen((std::string(ANIMATION_DIRECTORY "/") + a.name + ".pb").c_str(), "wb");
    TEST_ASSERT_NOT_NULL(f);
    fwrite(bytes, 1, stream.bytes_written, f);
    fclose(f);
}

// Two standing keyframes a second apart at a fixed ride height of 0; with `swing`, half a sine of 80 mm on leg 0's foot
// backward and leg 1's forward, which brings the two together from t = 0.375 s on.
void writeClip(const char* name, bool swing) {
    animation_Animation a = animation_Animation_init_zero;
    strncpy(a.name, name, sizeof(a.name) - 1);
    a.schema = anim::SCHEMA_VERSION;
    a.keyframes_count = 2;
    a.keyframes[1].time = 1.0f;
    a.has_ride_height = true;
    a.ride_height = 0.0f;
    if (swing) {
        a.overlays_count = 2;
        const uint32_t channels[2] = {1, 4};
        const float amplitudes[2] = {-80.0f, 80.0f};
        for (int i = 0; i < 2; ++i) {
            animation_Overlay& o = a.overlays[i];
            o.which_channel = animation_Overlay_foot_channel_tag;
            o.channel.foot_channel = channels[i];
            o.amplitude = amplitudes[i];
            o.frequency = 0.5f;
            o.start = 0.0f;
            o.end = 1.0f;
        }
    }
    writeAnimation(a);
}

Kinematics kin;

AnimationRunner& runner() {
    static AnimationRunner r;
    static const bool ready = r.begin(kin, STANCE);
    TEST_ASSERT_TRUE(ready);
    return r;
}

socket_message_AnimationReport validate(const char* name) {
    socket_message_AnimationReport report = socket_message_AnimationReport_init_zero;
    runner().validate(name, report, STANCE);
    return report;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_a_clip_that_brings_two_legs_together_is_refused_naming_the_frame_and_the_pair() {
    writeClip("touching", true);
    const socket_message_AnimationReport report = validate("touching");
    TEST_ASSERT_FALSE(report.ok);
    TEST_ASSERT_EQUAL_STRING("legs RF and RM touch at t=0.375 s on a +0 mm base", report.error);
    TEST_ASSERT_TRUE(report.clamped_mask & anim::LEGS_TOUCH);
}

void test_a_clip_that_keeps_the_legs_apart_validates() {
    writeClip("standing", false);
    const socket_message_AnimationReport report = validate("standing");
    TEST_ASSERT_TRUE(report.ok);
    TEST_ASSERT_EQUAL_STRING("", report.error);
    TEST_ASSERT_EQUAL_UINT32(0, report.clamped_mask);
}

// A clip that swings legs 0 and 1 45 deg into each other in a tenth of a second and back in one hundredth, then exits
// in one tick: the angles are held short of the touch and catch up with the stand at the servo's speed after the player
// has gone idle at stance. Until they have, the runner is busy, so a borrowed mode is not handed back mid catch-up, and
// its status reports LEGS_TOUCH; once they have, it is neither.
void test_the_runner_is_busy_and_reports_the_touch_until_the_held_angles_catch_up() {
    float stance[18];
    anim::poseToAngles(anim::Pose {}, kin, STANCE, stance);
    animation_Animation a = animation_Animation_init_zero;
    strncpy(a.name, "snap", sizeof(a.name) - 1);
    a.schema = anim::SCHEMA_VERSION;
    a.entry_time = 0.005f;
    a.exit_time = 0.005f;
    a.has_ride_height = true;
    a.ride_height = 0.0f;
    a.keyframes_count = 3;
    a.keyframes[1].time = 0.1f;
    a.keyframes[2].time = 0.11f;
    a.keyframes[1].legs_count = 6;
    for (int leg = 0; leg < 6; ++leg) {
        animation_LegTarget& t = a.keyframes[1].legs[leg];
        t.which_target = animation_LegTarget_joints_tag;
        t.target.joints = {stance[leg * 3] + (leg == 0   ? -45.0f
                                              : leg == 1 ? 45.0f
                                                         : 0.0f),
                           stance[leg * 3 + 1], stance[leg * 3 + 2]};
    }
    writeAnimation(a);

    BodyStateMsg live {};
    live.updateFeet(STANCE);
    AnimationRunner& r = runner();
    r.enter(live, 0.0f);
    AnimationCommandMsg play {};
    strncpy(play.name, "snap", sizeof(play.name) - 1);
    TEST_ASSERT_TRUE(r.requestPlay(play));
    bool caughtUpAfterIdle = false;
    for (int tick = 0; tick < 200; ++tick) {
        BodyStateMsg body;
        float angles[18];
        r.tick(0.005f, 0.0f, body, angles);
        socket_message_AnimationStatus status = socket_message_AnimationStatus_init_zero;
        EventBus<socket_message_AnimationStatus>::peek(status);
        float apart = 0.0f;
        for (int j = 0; j < 18; ++j) apart = fmaxf(apart, fabsf(angles[j] - stance[j]));
        if (!r.busy()) TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, apart);
        if (status.state == socket_message_AnimationState_ANIM_IDLE && apart > 1.0f) {
            TEST_ASSERT_TRUE(r.busy());
            TEST_ASSERT_TRUE(status.clamped_mask & anim::LEGS_TOUCH);
            caughtUpAfterIdle = true;
        }
    }
    TEST_ASSERT_TRUE(caughtUpAfterIdle);
    TEST_ASSERT_FALSE(r.busy());
    socket_message_AnimationStatus status = socket_message_AnimationStatus_init_zero;
    EventBus<socket_message_AnimationStatus>::peek(status);
    TEST_ASSERT_FALSE(status.clamped_mask & anim::LEGS_TOUCH);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_a_clip_that_brings_two_legs_together_is_refused_naming_the_frame_and_the_pair);
    RUN_TEST(test_a_clip_that_keeps_the_legs_apart_validates);
    RUN_TEST(test_the_runner_is_busy_and_reports_the_touch_until_the_held_angles_catch_up);
    return UNITY_END();
}
