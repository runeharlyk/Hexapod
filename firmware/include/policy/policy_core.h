#pragma once

// The learned policy's observation and action decoding: what the network sees and what its outputs do. The robot's
// PolicyRunner (policy_runner.h) runs it with the constants of the exported network's header; the training
// environment runs it through the native library (firmware/host/policy_api.cpp) with the constants it trains under,
// which export_policy.py then writes into that header. One implementation, so training and the robot cannot drift.

#include <math.h>
#include <gait.h>
#include <imu_frame.h>
#include <message_types.h>

namespace policy_layout {

constexpr int SENSOR_FRAME = 6;   // gravity(3) + gyro(3)
constexpr int GAIT_CHANNELS = 6;  // residual_gait: step_x, step_y, step_height, blend, cadence, body_zm
constexpr int FOOT_CHANNELS = 18; // 6 legs x [dx, dy, dz]

// Sensor history, rpy(3), previous joint angles(18), phase clock(2), dt(1), command(3), previous action.
constexpr int observationDim(int history, int actionDim) {
    return SENSOR_FRAME * history + 3 + 18 + 2 + 1 + 3 + actionDim;
}

} // namespace policy_layout

struct PolicyParams {
    int actionDim;    // FOOT_CHANNELS (residual_pure) or GAIT_CHANNELS + FOOT_CHANNELS (residual_gait)
    int obsHistory;   // sensor frames in the observation, newest first
    int histStride;   // control steps between them
    float actionTauS; // the output filter's time constant
    float footResidualMm;
    float stepXyMm, stepAngleRad;
    float stepHeightMinMm, stepHeightMaxMm;
    float phaseRateMin, phaseRateMax; // cycles/s
    float stepDepthMm;
    float bodyZmMm;                  // ride-height authority of residual_gait's sixth channel
    float bodyRideMm;                // the base gait's ride height, + raises the body
    float gaitDeltaGain[5];          // step_x, step_y, step_height, blend, cadence
    float triOffset[6], biOffset[6]; // leg phase offsets at blend 0 and 1
    float triStandFrac, biStandFrac;
};

// The IMU as Peripherals reports it.
struct ImuReading {
    float gravityUp[3]; // world up in the body frame, the DMP's convention
    float gyroRad[3];   // body rates
    float rpyRad[3];    // roll, pitch, yaw
};

// MAX_SPAN sensor frames and MAX_ACTION outputs are the capacity; params says how much of it a network uses.
template <int MAX_SPAN, int MAX_ACTION>
class PolicyCore {
  public:
    explicit PolicyCore(const PolicyParams &params) : p(params) {}

    const PolicyParams &params() const { return p; }
    int observationDim() const { return policy_layout::observationDim(p.obsHistory, p.actionDim); }
    bool fits() const {
        return span() <= MAX_SPAN && p.actionDim <= MAX_ACTION &&
               (p.actionDim == policy_layout::FOOT_CHANNELS ||
                p.actionDim == policy_layout::GAIT_CHANNELS + policy_layout::FOOT_CHANNELS);
    }

    // Phase 0, no previous action, the feet snapped to the stance the policy was trained around.
    void reset(GaitController &gait, const float stance[6][4]) {
        phase = 0.0f;
        dt = 0.0f;
        histCount = 0;
        for (float &c : cmd) c = 0.0f;
        for (float &a : action) a = 0.0f;
        gait.snapDefaultFootTarget(stance);
    }

    // Body-frame velocity: forward (m/s), leftward (m/s) and counter-clockwise yaw rate (rad/s).
    void setCommand(float vx, float vy, float yawRate) {
        cmd[0] = vx;
        cmd[1] = vy;
        cmd[2] = yawRate;
    }
    const float *command() const { return cmd; }

    // Records the sensor frame and writes the observation the next action is chosen from. dt is the time since the
    // last observation, and apply() advances the gait phase by it.
    void observe(const ImuReading &imu, const float jointAnglesDeg[18], float dtS, float *obs) {
        dt = dtS;
        for (int k = span() - 1; k > 0; --k)
            for (int j = 0; j < FRAME; ++j) hist[k][j] = hist[k - 1][j];
        for (int j = 0; j < 3; ++j) hist[0][j] = -imu.gravityUp[j]; // the policy takes world down
        for (int j = 0; j < 3; ++j) hist[0][3 + j] = imu.gyroRad[j];
        if (histCount < span()) histCount++;

        int o = 0;
        for (int f = 0; f < p.obsHistory; ++f) {
            const int age = f * p.histStride;
            // Before the buffer fills, repeat the oldest frame rather than feeding zeros: a zero gravity vector is not
            // a pose the policy ever saw in training.
            const int idx = age < histCount ? age : histCount - 1;
            for (int j = 0; j < FRAME; ++j) obs[o++] = hist[idx][j];
        }
        obs[o++] = OBS_SIGN_ROLL * imu.rpyRad[0];
        obs[o++] = OBS_SIGN_PITCH * imu.rpyRad[1];
        obs[o++] = OBS_SIGN_YAW * imu.rpyRad[2];
        for (int i = 0; i < 18; ++i) obs[o++] = jointAnglesDeg[i] * (float)M_PI / 180.0f;
        obs[o++] = sinf(2.0f * (float)M_PI * phase);
        obs[o++] = cosf(2.0f * (float)M_PI * phase);
        obs[o++] = dt;
        for (float c : cmd) obs[o++] = c;
        for (int i = 0; i < p.actionDim; ++i) obs[o++] = action[i];
    }

    // Applies the network's raw output to the base gait's normalized params (from analytic_gait): clips it to
    // [-1, 1] and low-pass filters it, adds the gait deltas, generates the feet at the advanced phase and adds the
    // foot residuals. The caller then solves the feet into joint angles.
    void apply(const float *raw, const float baseGait[6], GaitController &gait, BodyStateMsg &body) {
        const float alpha = expf(-dt / p.actionTauS);
        for (int i = 0; i < p.actionDim; ++i) action[i] = alpha * action[i] + (1.0f - alpha) * clip1(raw[i]);

        float a[6];
        for (int i = 0; i < 6; ++i) a[i] = baseGait[i];
        const int residualBase = p.actionDim - policy_layout::FOOT_CHANNELS;
        float zmDelta = 0.0f;
        if (residualBase == policy_layout::GAIT_CHANNELS) {
            a[0] = clip1(a[0] + p.gaitDeltaGain[0] * action[0]); // step_x
            a[1] = clip1(a[1] + p.gaitDeltaGain[1] * action[1]); // step_y
            a[3] = clip1(a[3] + p.gaitDeltaGain[2] * action[2]); // step_height
            a[4] = clip1(a[4] + p.gaitDeltaGain[3] * action[3]); // tripod<->bipod
            a[5] = clip1(a[5] + p.gaitDeltaGain[4] * action[4]); // cadence
            zmDelta = action[5] * p.bodyZmMm;
        }
        applyGait(a, gait, body, zmDelta);
        for (int i = 0; i < 6; ++i)
            for (int k = 0; k < 3; ++k) body.feet[i][k] += action[residualBase + i * 3 + k] * p.footResidualMm;
    }

    // What the last apply() commanded.
    struct Applied {
        float blend, stepHeightMm, phaseRate, standFrac;
    };
    const Applied &applied() const { return last; }
    // The filtered action the last apply() used, which the next observation reports.
    const float *appliedAction() const { return action; }
    float gaitPhase() const { return phase; }

  private:
    static constexpr int FRAME = policy_layout::SENSOR_FRAME;

    // The policy was trained on the simulation's body-frame angles, which the DMP's differ from (imu_frame.h);
    // gravity and gyro pass through (gravity already negated: the DMP's is world up, the policy's world down).
    static constexpr float OBS_SIGN_ROLL = imu_frame::ROLL_SIGN;
    static constexpr float OBS_SIGN_PITCH = imu_frame::PITCH_SIGN;
    static constexpr float OBS_SIGN_YAW = imu_frame::YAW_SIGN;

    static float clip1(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }
    int span() const { return (p.obsHistory - 1) * p.histStride + 1; }

    // Decodes the normalized gait params, blends tripod<->bipod, advances the phase by dt and generates the feet.
    void applyGait(const float a[6], GaitController &gait, BodyStateMsg &body, float zmDelta) {
        gaitState.step_x = a[0] * p.stepXyMm;
        gaitState.step_y = a[1] * p.stepXyMm;
        gaitState.step_angle = a[2] * p.stepAngleRad;
        gaitState.step_height = p.stepHeightMinMm + (a[3] + 1.0f) * 0.5f * (p.stepHeightMaxMm - p.stepHeightMinMm);
        gaitState.step_depth = p.stepDepthMm;
        const float blend = (a[4] + 1.0f) * 0.5f;
        for (int i = 0; i < 6; ++i) gaitState.offset[i] = (1.0f - blend) * p.triOffset[i] + blend * p.biOffset[i];
        gaitState.stand_frac = (1.0f - blend) * p.triStandFrac + blend * p.biStandFrac;
        const float phaseRate = p.phaseRateMin + (a[5] + 1.0f) * 0.5f * (p.phaseRateMax - p.phaseRateMin);
        body.zm = -(p.bodyRideMm + zmDelta); // negative zm raises the body
        phase = fmodf(phase + dt * phaseRate, 1.0f);
        gait.setPhase(phase);
        gait.generateFeet(gaitState, body);
        last = {blend, gaitState.step_height, phaseRate, gaitState.stand_frac};
    }

    PolicyParams p;
    // Newest frame at index 0, walking backwards in time.
    float hist[MAX_SPAN][FRAME] = {};
    int histCount = 0;
    float phase = 0.0f;
    float dt = 0.0f;
    float cmd[3] = {0.0f, 0.0f, 0.0f};
    // The filtered action: what was applied, and what the next observation reports.
    float action[MAX_ACTION] = {};
    gait_state_t gaitState = {
        15, 0, 0, 0, 1, 0.002f, default_stand_frac, GaitType::TRI_GATE, {0, 0.52f, 0.08f, 0.58f, 0.16f, 0.66f}};
    Applied last = {};
};
