// The animation evaluator and player as a C interface for the app's animation editor (app/src/lib/animation/engine.ts):
// the code the robot's animation runner plays clips with, so the editor previews a clip exactly as the robot plays it.
// A session holds one clip, its parameter values, the standing feet and a player. Poses and angles cross the boundary
// through the session's io buffer:
//   pose    io[0, 30): the body (roll, pitch, yaw, x, y, z), then per leg whether it is a joint target and its 3 values
//   angles  io[30, 48): 18 joint angles in degrees, coxa, femur and tibia per leg in leg order

#include <host_api.h>
#include <animation/animation.h>
#include <animation/animation_codec.h>
#include <pb_decode.h>
#include <vector>

namespace {

constexpr int POSE = 0;
constexpr int ANGLES = 30;
constexpr int IO_SIZE = 48;

struct Session {
    Kinematics kin;
    anim::Player player {kin};
    anim::Clip clip;
    float stance[6][4] = {};
    anim::ParamValue values[anim::PARAM_COUNT] = {};
    int valueCount = 0;
    std::vector<uint8_t> encoded;
    animation_Animation decoded = animation_Animation_init_zero;
    float io[IO_SIZE] = {};

    Session() { player.setStance(stance); }

    anim::Pose pose() const {
        anim::Pose p;
        for (int a = 0; a < 6; ++a) p.body[a] = io[POSE + a];
        for (int i = 0; i < 6; ++i) {
            const float *leg = &io[POSE + 6 + i * 4];
            p.legs[i] = {leg[0] != 0.0f, {leg[1], leg[2], leg[3]}};
        }
        return p;
    }

    void setPose(const anim::Pose &p) {
        for (int a = 0; a < 6; ++a) io[POSE + a] = p.body[a];
        for (int i = 0; i < 6; ++i) {
            float *leg = &io[POSE + 6 + i * 4];
            leg[0] = p.legs[i].joints ? 1.0f : 0.0f;
            for (int k = 0; k < 3; ++k) leg[1 + k] = p.legs[i].v[k];
        }
    }
};

Session &session(void *handle) { return *static_cast<Session *>(handle); }

} // namespace

extern "C" {

API void *animCreate() { return new Session(); }

API void animDestroy(void *handle) { delete static_cast<Session *>(handle); }

API float *animIo(void *handle) { return session(handle).io; }

// The standing feet every offset is relative to, 4 values per leg; the caller writes them before evaluating.
API float *animStance(void *handle) { return &session(handle).stance[0][0]; }

// A buffer of size bytes for an encoded animation.proto Animation, which animLoad then decodes.
API uint8_t *animClipBuffer(void *handle, int size) {
    Session &s = session(handle);
    s.encoded.resize(size);
    return s.encoded.data();
}

// Decodes the encoded clip into the session's clip, unvalidated, as the editor previews documents still in progress.
// Returns the decoder's error, or null. A playing player keeps the clip it was given and must not see a reload.
API const char *animLoad(void *handle, int size) {
    Session &s = session(handle);
    pb_istream_t stream = pb_istream_from_buffer(s.encoded.data(), size);
    if (!pb_decode(&stream, animation_Animation_fields, &s.decoded)) return PB_GET_ERROR(&stream);
    anim::fromProto(s.decoded, s.clip);
    return nullptr;
}

API void animClearParams(void *handle) { session(handle).valueCount = 0; }

API void animSetParam(void *handle, int id, float value) {
    Session &s = session(handle);
    if (s.valueCount < anim::PARAM_COUNT) s.values[s.valueCount++] = {id, value};
}

// The clip's pose at t into the io pose.
API void animEvaluate(void *handle, float t, float baseZ) {
    Session &s = session(handle);
    float params[anim::PARAM_COUNT];
    anim::resolveParams(s.clip, s.values, s.valueCount, params);
    anim::Pose pose;
    anim::evaluate(s.clip, params, t, s.kin, s.stance, pose, baseZ);
    s.setPose(pose);
}

// The io pose's servo angles into the io angles; returns the mask of joints at a limit or on an unreachable foot.
API unsigned animPoseToAngles(void *handle) {
    Session &s = session(handle);
    return anim::poseToAngles(s.pose(), s.kin, s.stance, &s.io[ANGLES]);
}

// The joints of the io pose's foot leg, solved on the io pose's body at the base, into the first 3 io angles.
API void animLegJoints(void *handle, int leg, float baseZ) {
    Session &s = session(handle);
    const anim::Pose pose = s.pose();
    anim::legJointsDeg(s.kin, pose.body, pose.legs[leg].v, leg, s.stance, &s.io[ANGLES], baseZ);
}

// Plays the session's clip with its parameter values, entering from the io pose when live is set, else from the pose
// the player last output.
API void animPlay(void *handle, bool live, float baseZ) {
    Session &s = session(handle);
    const anim::Pose pose = s.pose();
    s.player.play(&s.clip, s.values, s.valueCount, live ? &pose : nullptr, baseZ);
}

API void animStop(void *handle, float baseZ) { session(handle).player.stop(baseZ); }

// Advances the player by dt seconds and writes its pose into the io pose.
API void animUpdate(void *handle, float dt, float baseZ) {
    Session &s = session(handle);
    s.setPose(s.player.update(dt, baseZ));
}

API void animLastPose(void *handle) {
    Session &s = session(handle);
    s.setPose(s.player.lastPose());
}

API int animState(void *handle) { return static_cast<int>(session(handle).player.state()); }

API float animTime(void *handle) { return session(handle).player.t(); }

API float animBlendFraction(void *handle) { return session(handle).player.blendFraction(); }

API float animEntryBase(void *handle) { return session(handle).player.entryBase(); }

API float animEase(int kind, float t) { return anim::easeValue(kind, t); }
}
