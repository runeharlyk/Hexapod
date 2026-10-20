// The robot's WALK_NN control path as a C interface for the training environment (simulation/src/sim/
// firmware_policy.py): the self-levelling, PolicyCore's observation and action decoding, the gait and the inverse
// kinematics the robot runs between an IMU reading and its joint targets. Only the network is outside: the
// environment's policy chooses the action from the observation this returns. One session per environment.

#include <algorithm>
#include <cmath>
#include <host_api.h>
#include <kinematics.h>
#include <gait.h>
#include <message_types.h>
#include <policy/policy_core.h>
#include <self_level.h>

namespace {

constexpr float CONTROL_TICK_S = 0.005f; // main.cpp's control task period
constexpr int MAX_SPAN = 64;             // sensor frames a training run's history may span
constexpr int MAX_ACTION = policy_layout::GAIT_CHANNELS + policy_layout::FOOT_CHANNELS;

// The stance of fd 0, gait_speed.h's DEFAULT_STANCE, which MotionService resets the policy on and walks WALK_NN on.
constexpr float STANCE[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

struct Session {
    Kinematics kinematics;
    GaitController gait;
    BodyStateMsg body;
    PolicyCore<MAX_SPAN, MAX_ACTION> core;
    ImuReading imu = {};
    float dt = 0.0f;
    float angles[18] = {};
    float state[6] = {};

    explicit Session(const PolicyParams &params) : core(params) {}
};

Session &session(void *handle) { return *static_cast<Session *>(handle); }

} // namespace

extern "C" {

// The size of PolicyParams, for the caller to check its copy of the layout against.
API int policyParamsSize() { return sizeof(PolicyParams); }

// A session for these constants, or null when PolicyCore cannot run them.
API void *policyCreate(const PolicyParams *params) {
    Session *s = new Session(*params);
    if (s->core.fits()) return s;
    delete s;
    return nullptr;
}

API void policyDestroy(void *handle) { delete static_cast<Session *>(handle); }

// The robot standing level on its stance, as WALK_NN starts: the policy reset and the joints solved for that stance.
API void policyReset(void *handle) {
    Session &s = session(handle);
    s.gait = GaitController();
    s.body = BodyStateMsg();
    s.body.updateFeet(STANCE);
    s.core.reset(s.gait, STANCE);
    s.kinematics.inverseKinematics(s.body, s.angles);
}

// Body-frame velocity: forward (m/s), leftward (m/s) and counter-clockwise yaw rate (rad/s).
API void policySetCommand(void *handle, float vx, float vy, float yawRate) {
    session(handle).core.setCommand(vx, vy, yawRate);
}

// Writes the observation the next action is chosen from, for an IMU reading taken dt seconds after the last one.
// Returns its length.
API int policyObserve(void *handle, const ImuReading *imu, float dt, float *obs) {
    Session &s = session(handle);
    s.imu = *imu;
    s.dt = dt;
    s.core.observe(s.imu, s.angles, dt, obs);
    return s.core.observationDim();
}

// Applies the network's raw output on the base gait's normalized params: the control ticks since the observation
// level the body against the IMU, then the action sets the feet and the inverse kinematics solves them. Returns the
// 18 joint targets in degrees, coxa, femur and tibia per leg in leg order.
API const float *policyAct(void *handle, const float *action, const float *baseGait) {
    Session &s = session(handle);
    const int ticks = (int)lroundf(s.dt / CONTROL_TICK_S);
    for (int i = 0; i < ticks; ++i) levelAttitude(s.body, 0.f, 0.f, s.imu.rpyRad[0], s.imu.rpyRad[1]);
    s.core.apply(action, baseGait, s.gait, s.body);
    s.kinematics.inverseKinematics(s.body, s.angles);
    return s.angles;
}

// The filtered action the last policyAct applied, actionDim values.
API const float *policyAction(void *handle) { return session(handle).core.appliedAction(); }

// What the last action commanded: tripod<->bipod blend, step height (mm), cadence (cycles/s), stance fraction, gait
// phase and body zm (mm).
API const float *policyState(void *handle) {
    Session &s = session(handle);
    const auto &applied = s.core.applied();
    const float state[6] = {applied.blend,     applied.stepHeightMm, applied.phaseRate,
                            applied.standFrac, s.core.gaitPhase(),   s.body.zm};
    std::copy_n(state, 6, s.state);
    return s.state;
}
}
