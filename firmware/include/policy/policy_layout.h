#pragma once

#include <policy/hexapod_policy.h>

// The observation and action layout PolicyRunner implements, checked against the exported header at compile
// time: a header generated for another control mode, history length or contact input fails the build instead of
// feeding the network a misaligned observation or leaving action channels unapplied.
namespace policy_layout {

constexpr int SENSOR_FRAME = 6;   // gravity(3) + gyro(3)
constexpr int GAIT_CHANNELS = 6;  // residual_gait: step_x, step_y, step_height, blend, cadence, body_zm
constexpr int FOOT_CHANNELS = 18; // 6 legs x [dx, dy, dz]

// Sensor history, rpy(3), previous joint angles(18), phase clock(2), dt(1), command(3), previous action.
constexpr int observationDim(int history, int actionDim) {
    return SENSOR_FRAME * history + 3 + 18 + 2 + 1 + 3 + actionDim;
}

static_assert(hexapod_policy::ACT_DIM == FOOT_CHANNELS || hexapod_policy::ACT_DIM == GAIT_CHANNELS + FOOT_CHANNELS,
              "hexapod_policy.h was exported for a control mode PolicyRunner does not decode: it applies "
              "residual_pure (18 foot residuals) or residual_gait (6 gait channels + 18 foot residuals)");
static_assert(hexapod_policy::OBS_DIM == observationDim(hexapod_policy::OBS_HISTORY, hexapod_policy::ACT_DIM),
              "hexapod_policy.h expects an observation PolicyRunner does not build (history length or contact "
              "inputs differ from the sensor-frame, rpy, joints, phase, dt, command and previous-action stack)");

} // namespace policy_layout
