#pragma once

#include <features.h>

#if FT_ENABLED(USE_POLICY)

#include <math.h>
#include <gait.h>
#include <message_types.h>
#include <peripherals/peripherals.h>
#include <policy/hexapod_policy.h>
#include <utils/timing.h>

// Runs the learned residual policy (simulation/export_policy.py output) on top of the
// analytic gait: gait params + phase come from hexapod_policy::analytic_gait, the network
// adds bounded per-foot residuals. Mirrors HexapodMjEnv control_mode=residual_pure — any
// change here must stay in lockstep with simulation/src/envs/hexapod_mj_env.py.
class PolicyRunner {
  public:
    // Verify the baked-in network reproduces its golden test vector (call once at boot).
    static bool verify() {
        const float err = hexapod_policy::selfCheck();
        ESP_LOGI("PolicyRunner", "policy self-check max err %.3e (%s)", err, err < 1e-4f ? "OK" : "FAIL");
        return err < 1e-4f;
    }

    // Mirrors the env reset: phase 0, zero previous action, feet snapped to the default
    // stance the policy was trained around (foot-distance scaling does not apply in NN mode).
    void reset(GaitController &gait, const float trained_feet[6][4]) {
        phase = 0.0f;
        tick = 0;
        cmd[0] = cmd[1] = cmd[2] = 0.0f;
        for (int i = 0; i < hexapod_policy::ACT_DIM; ++i) prevAction[i] = 0.0f;
        histCount = 0;
        gait.snapDefaultFootTarget(trained_feet);
    }

    // Joystick [-1,1] -> the command ranges the policy was trained on.
    // Forward stick (-lx, same convention as the classic WALK mapping) reaches CMD_VX_MAX;
    // backward reaches CMD_VX_MIN (training range is asymmetric).
    void setCommand(CommandMsg const &c) {
        const float fwd = -c.lx;
        cmd[0] = fwd * (fwd >= 0.0f ? hexapod_policy::CMD_VX_MAX : -hexapod_policy::CMD_VX_MIN);
        cmd[1] = c.ly * hexapod_policy::CMD_VY_MAX;
        cmd[2] = c.rx * hexapod_policy::CMD_YAW_MAX;
    }

    // Call every control tick (5 ms); the network runs every 4th tick (CONTROL_DT = 20 ms,
    // the training rate) and joint targets hold in between, exactly like the sim.
    // Returns true when body.feet was rewritten (caller then runs IK).
    bool update(Peripherals *peripherals, GaitController &gait, BodyStateMsg &body, const float jointAnglesDeg[18]) {
        if (tick++ % TICKS_PER_INFERENCE != 0) return false;

        pushSensorFrame(peripherals);

        float obs[hexapod_policy::OBS_DIM];
        buildObservation(obs, peripherals, jointAnglesDeg);
        float action[hexapod_policy::ACT_DIM];
        hexapod_policy::infer(obs, action);

        namespace hp = hexapod_policy;
        float gaitAction[6];
        hp::analytic_gait(cmd, gaitAction);
        // residual_gait spends its last 6 channels on the gait itself. The deltas are added to the
        // analytic map's NORMALIZED params and re-clipped, exactly as the env does.
        float zmDelta = 0.0f;
        if (hp::ACT_DIM >= 24) {
            const float *d = action + 18;
            gaitAction[0] = clip1(gaitAction[0] + hp::GAIT_DELTA_GAIN[0] * d[0]);  // step_x
            gaitAction[1] = clip1(gaitAction[1] + hp::GAIT_DELTA_GAIN[1] * d[1]);  // step_y
            gaitAction[3] = clip1(gaitAction[3] + hp::GAIT_DELTA_GAIN[2] * d[2]);  // step_height
            gaitAction[4] = clip1(gaitAction[4] + hp::GAIT_DELTA_GAIN[3] * d[3]);  // tripod<->bipod
            gaitAction[5] = clip1(gaitAction[5] + hp::GAIT_DELTA_GAIN[4] * d[4]);  // cadence
            zmDelta = d[5] * hp::BODY_ZM_MM;
        }
        applyGaitParams(gaitAction, gait, body, zmDelta);

        for (int i = 0; i < 6; ++i) {
            body.feet[i][0] += action[i * 3 + 0] * hp::FOOT_RESIDUAL_MM;
            body.feet[i][1] += action[i * 3 + 1] * hp::FOOT_RESIDUAL_MM;
            body.feet[i][2] += action[i * 3 + 2] * hp::FOOT_RESIDUAL_MM;
        }
        for (int i = 0; i < hexapod_policy::ACT_DIM; ++i) prevAction[i] = action[i];
        return true;
    }

  private:
    static constexpr int TICKS_PER_INFERENCE = 4;  // 200 Hz control loop -> 50 Hz policy
    static constexpr int FRAME = 6;                // gravity(3) + gyro(3) per sensor frame
    static constexpr int HIST_SPAN = (hexapod_policy::OBS_HISTORY - 1) * hexapod_policy::HIST_STRIDE + 1;

    static float clip1(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }

    // Newest frame at index 0, walking backwards in time.
    float hist[HIST_SPAN][FRAME] = {{0.0f}};
    int histCount = 0;

    void pushSensorFrame(Peripherals *p) {
        for (int k = HIST_SPAN - 1; k > 0; --k) {
            for (int j = 0; j < FRAME; ++j) hist[k][j] = hist[k - 1][j];
        }
        float grav[3], gyro[3];
        p->gravityBody(grav);
        p->gyroRad(gyro);
        hist[0][0] = -grav[0];  // sim gravity is world-DOWN in body frame; DMP gravity is world-UP
        hist[0][1] = -grav[1];
        hist[0][2] = -grav[2];
        hist[0][3] = gyro[0];
        hist[0][4] = gyro[1];
        hist[0][5] = gyro[2];
        if (histCount < HIST_SPAN) histCount++;
    }

    float phase = 0.0f;
    uint32_t tick = 0;
    float cmd[3] = {0.0f, 0.0f, 0.0f};
    float prevAction[hexapod_policy::ACT_DIM] = {0.0f};
    gait_state_t gait_state = {15, 0, 0, 0, 1, 0.002, default_stand_frac, GaitType::TRI_GATE,
                               {0, 0.52, 0.08, 0.58, 0.16, 0.66}};

    // The policy was trained on MuJoCo conventions (right-handed, aerospace rpy: pitch
    // positive = nose DOWN). The I2Cdev DMP formulas differ: dmpGetYawPitchRoll pitch is
    // positive nose-UP (atan2(g.x, ...)) and yaw is mirrored, while roll matches. For a
    // sensor mounted aligned with the model frame that means pitch and yaw must be negated
    // and roll/gravity/gyro pass through (gravity already negated: DMP is world-up, sim
    // wants world-down). STAND-mode IMU compensation working on hardware confirms the
    // axis PAIRING (no 90-degree mounting); flip the signs below if the bench test fails.
    //
    // BENCH TEST (serial, raise CORE_DEBUG_LEVEL to see the obs log): hold the robot,
    //   nose (leg 1 / +x side) tilted DOWN ~15 deg -> obs[7] = +0.26, obs[0] = +0.26
    //   leg 0/3 (+y) side tilted DOWN ~15 deg      -> obs[6] = -0.26, obs[1] = +0.26
    //   turning left (counter-clockwise from above) -> obs[5] (gyro z) positive
    static constexpr float OBS_SIGN_ROLL = +1.0f;
    static constexpr float OBS_SIGN_PITCH = -1.0f;
    static constexpr float OBS_SIGN_YAW = -1.0f;

    void buildObservation(float *obs, Peripherals *p, const float jointAnglesDeg[18]) {
        namespace hp = hexapod_policy;
        int o = 0;
        for (int f = 0; f < hp::OBS_HISTORY; ++f) {
            const int age = f * hp::HIST_STRIDE;
            // Before the buffer fills, repeat the oldest frame rather than feeding zeros: a zero
            // gravity vector is not a pose the policy ever saw in training.
            const int idx = age < histCount ? age : (histCount > 0 ? histCount - 1 : 0);
            for (int j = 0; j < FRAME; ++j) obs[o++] = hist[idx][j];
        }
        obs[o++] = OBS_SIGN_ROLL * p->angleX();
        obs[o++] = OBS_SIGN_PITCH * p->angleY();
        obs[o++] = OBS_SIGN_YAW * p->angleZ();
        constexpr float DEG2RAD = (float)M_PI / 180.0f;
        for (int i = 0; i < 18; ++i) obs[o++] = jointAnglesDeg[i] * DEG2RAD;
        obs[o++] = sinf(2.0f * (float)M_PI * phase);
        obs[o++] = cosf(2.0f * (float)M_PI * phase);
        obs[o++] = cmd[0];
        obs[o++] = cmd[1];
        obs[o++] = cmd[2];
        for (int i = 0; i < hp::ACT_DIM; ++i) obs[o++] = prevAction[i];
        if (o != hp::OBS_DIM) {
            ESP_LOGE("PolicyRunner", "observation is %d floats, policy expects %d", o, hp::OBS_DIM);
        }
        EXECUTE_EVERY_N_MS(1000, ESP_LOGD("PolicyRunner",
                                          "obs grav[%.2f %.2f %.2f] gyro[%.2f %.2f %.2f] rpy[%.2f %.2f %.2f]",
                                          obs[0], obs[1], obs[2], obs[3], obs[4], obs[5], obs[6], obs[7], obs[8]));
    }

    // Mirrors HexapodMjEnv._apply_gait_params: decode normalized gait actions, blend
    // tripod<->bipod, advance the policy-owned phase, generate feet at that phase.
    void applyGaitParams(const float a[6], GaitController &gait, BodyStateMsg &body, float zmDelta = 0.0f) {
        namespace hp = hexapod_policy;
        gait_state.step_x = a[0] * hp::PG_STEP_XY;
        gait_state.step_y = a[1] * hp::PG_STEP_XY;
        gait_state.step_angle = a[2] * hp::PG_STEP_ANGLE;
        gait_state.step_height = hp::PG_HEIGHT_MIN + (a[3] + 1.0f) * 0.5f * (hp::PG_HEIGHT_MAX - hp::PG_HEIGHT_MIN);
        gait_state.step_depth = hp::STEP_DEPTH_MM;
        const float blend = (a[4] + 1.0f) * 0.5f;
        for (int i = 0; i < 6; ++i) {
            gait_state.offset[i] = (1.0f - blend) * hp::TRI_OFFSET[i] + blend * hp::BI_OFFSET[i];
        }
        gait_state.stand_frac = (1.0f - blend) * hp::TRI_STAND_FRAC + blend * hp::BI_STAND_FRAC;
        const float phase_rate = hp::PG_PHASE_RATE_MIN + (a[5] + 1.0f) * 0.5f * (hp::PG_PHASE_RATE_MAX - hp::PG_PHASE_RATE_MIN);
        body.zm = -(hp::BODY_RIDE_MM + zmDelta);  // firmware sign: negative zm raises the body
        phase = fmodf(phase + hp::CONTROL_DT * phase_rate, 1.0f);
        gait.setPhase(phase);
        gait.generateFeet(gait_state, body);
    }
};

#endif  // FT_ENABLED(USE_POLICY)
