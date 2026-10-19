#pragma once

#include <feature_flags.h>

#if FT_ENABLED(USE_POLICY)

#include <esp_timer.h>
#include <gait.h>
#include <message_types.h>
#include <peripherals/peripherals.h>
#include <policy/hexapod_policy.h>
#include <policy/policy_core.h>
#include <policy/policy_layout.h>
#include <utils/timing.h>

// Runs the learned policy (simulation/export_policy.py output) on top of the analytic gait: the network chooses
// from PolicyCore's observation, and PolicyCore applies its output, with the constants the network was trained
// under. The training environment runs the same PolicyCore (firmware/host/policy_api.cpp).
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
        tick = 0;
        lastInferenceUs = 0;
        core.reset(gait, trained_feet);
    }

    // The stick-to-command conversion is policy_layout::stickToPolicyCommand; PolicyCore::setCommand takes the frame
    // the policy was trained on.
    void setCommand(CommandMsg const &c) {
        const policy_layout::PolicyCommand cmd = policy_layout::stickToPolicyCommand(c.lx, c.ly, c.rx);
        core.setCommand(cmd.vx, cmd.vy, cmd.yawRate);
    }

    // Call every control tick (5 ms); the network runs every 4th tick (CONTROL_DT = 20 ms,
    // the training rate) and joint targets hold in between, exactly like the sim.
    // Returns true when body.feet was rewritten (caller then runs IK).
    bool update(Peripherals *peripherals, GaitController &gait, BodyStateMsg &body, const float jointAnglesDeg[18]) {
        if (tick++ % TICKS_PER_INFERENCE != 0) return false;

        namespace hp = hexapod_policy;
        const int64_t nowUs = esp_timer_get_time();
        const float dt = lastInferenceUs == 0 ? hp::CONTROL_DT : (float)(nowUs - lastInferenceUs) * 1e-6f;
        lastInferenceUs = nowUs;

        ImuReading imu;
        peripherals->gravityBody(imu.gravityUp);
        peripherals->gyroRad(imu.gyroRad);
        imu.rpyRad[0] = peripherals->angleX();
        imu.rpyRad[1] = peripherals->angleY();
        imu.rpyRad[2] = peripherals->angleZ();
        float obs[hp::OBS_DIM];
        core.observe(imu, jointAnglesDeg, dt, obs);
        EXECUTE_EVERY_N_MS(
            1000, ESP_LOGD("PolicyRunner", "obs grav[%.2f %.2f %.2f] gyro[%.2f %.2f %.2f] rpy[%.2f %.2f %.2f]", obs[0],
                           obs[1], obs[2], obs[3], obs[4], obs[5], obs[6], obs[7], obs[8]));

        float action[hp::ACT_DIM];
        hp::infer(obs, action);
        float baseGait[6];
        hp::analytic_gait(core.command(), baseGait);
        core.apply(action, baseGait, gait, body);
        return true;
    }

  private:
    static constexpr int TICKS_PER_INFERENCE = 4; // 200 Hz control loop -> 50 Hz policy

    uint32_t tick = 0;
    int64_t lastInferenceUs = 0;
    PolicyCore<policy_layout::HISTORY_SPAN, hexapod_policy::ACT_DIM> core {policy_layout::exportedParams()};
};

#endif // FT_ENABLED(USE_POLICY)
