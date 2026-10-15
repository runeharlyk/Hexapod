#pragma once

#include <policy/hexapod_policy.h>
#include <policy/policy_core.h>

// The exported header checked against the layout PolicyCore implements, at compile time: a header generated for
// another control mode or contact input fails the build instead of feeding the network a misaligned observation or
// leaving action channels unapplied.
namespace policy_layout {

static_assert(hexapod_policy::ACT_DIM == FOOT_CHANNELS || hexapod_policy::ACT_DIM == GAIT_CHANNELS + FOOT_CHANNELS,
              "hexapod_policy.h was exported for a control mode PolicyCore does not decode: it applies "
              "residual_pure (18 foot residuals) or residual_gait (6 gait channels + 18 foot residuals)");
static_assert(hexapod_policy::OBS_DIM == observationDim(hexapod_policy::OBS_HISTORY, hexapod_policy::ACT_DIM),
              "hexapod_policy.h expects an observation PolicyCore does not build (contact inputs, or a history "
              "length that disagrees with OBS_DIM)");

// The sensor frames the exported network's history spans.
constexpr int HISTORY_SPAN = (hexapod_policy::OBS_HISTORY - 1) * hexapod_policy::HIST_STRIDE + 1;

// The constants the network was trained under, as export_policy.py wrote them.
inline PolicyParams exportedParams() {
    namespace hp = hexapod_policy;
    PolicyParams p = {};
    p.actionDim = hp::ACT_DIM;
    p.obsHistory = hp::OBS_HISTORY;
    p.histStride = hp::HIST_STRIDE;
    p.actionTauS = hp::ACTION_TAU_S;
    p.footResidualMm = hp::FOOT_RESIDUAL_MM;
    p.stepXyMm = hp::PG_STEP_XY;
    p.stepAngleRad = hp::PG_STEP_ANGLE;
    p.stepHeightMinMm = hp::PG_HEIGHT_MIN;
    p.stepHeightMaxMm = hp::PG_HEIGHT_MAX;
    p.phaseRateMin = hp::PG_PHASE_RATE_MIN;
    p.phaseRateMax = hp::PG_PHASE_RATE_MAX;
    p.stepDepthMm = hp::STEP_DEPTH_MM;
    p.bodyZmMm = hp::BODY_ZM_MM;
    p.bodyRideMm = hp::BODY_RIDE_MM;
    for (int i = 0; i < 5; ++i) p.gaitDeltaGain[i] = hp::GAIT_DELTA_GAIN[i];
    for (int i = 0; i < 6; ++i) {
        p.triOffset[i] = hp::TRI_OFFSET[i];
        p.biOffset[i] = hp::BI_OFFSET[i];
    }
    p.triStandFrac = hp::TRI_STAND_FRAC;
    p.biStandFrac = hp::BI_STAND_FRAC;
    return p;
}

} // namespace policy_layout
