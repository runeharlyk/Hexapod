#pragma once

#include <utils/math_utils.h>
#include <algorithm>
#include <array>
#include <functional>
#include <kinematics.h>
#include <message_types.h>

static constexpr float default_offset[6] = {0, 0.52, 0.08, 0.58, 0.16, 0.66};
static constexpr float default_stand_frac = 3.1 / 6;

struct gait_state_t {
    float step_height, step_x, step_z, step_angle, step_speed, step_depth, stand_frac;
    GaitType gait_type;
    float offset[6];
};

class GaitController {
  private:
    static constexpr float MAX_STEP_LENGTH_MM = 70.0f;
    static constexpr float MIN_STEP_LENGTH_MM = 12.0f;
    static constexpr float FOOT_PATH_MAX_SPEED_MM_S = 320.0f;
    static constexpr float BODY_SPEED_SAFETY = 0.85f;
    static constexpr float AUTO_RIPPLE_MAX_INPUT = 0.3f;
    static constexpr float AUTO_TRI_MAX_INPUT = 0.7f;
    static constexpr float MAX_COMMAND_TRANSLATION_MM = 141.421356f;
    static constexpr float MAX_COMMAND_TURN_RAD = 0.8f;
    static constexpr float TURN_EQUIVALENT_STEP_MM = 55.0f;
    static constexpr float REPOSITION_PHASE_SPEED = 0.9f;
    float autoCommandState = 0.0f;

    float phase = 0.0f;
    float defaultPosition[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                   {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};
    float targetDefaultPosition[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                         {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};
    float swingStartPosition[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                      {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};
    bool footWasSwinging[6] = {false, false, false, false, false, false};
    static constexpr uint8_t BEZIER_POINTS = 12;
    static constexpr std::array<float, BEZIER_POINTS> COMBINATORIAL_VALUES = {
        combinatorial_constexpr(11, 0),  // 1
        combinatorial_constexpr(11, 1),  // 11
        combinatorial_constexpr(11, 2),  // 55
        combinatorial_constexpr(11, 3),  // 165
        combinatorial_constexpr(11, 4),  // 330
        combinatorial_constexpr(11, 5),  // 462
        combinatorial_constexpr(11, 6),  // 462
        combinatorial_constexpr(11, 7),  // 330
        combinatorial_constexpr(11, 8),  // 165
        combinatorial_constexpr(11, 9),  // 55
        combinatorial_constexpr(11, 10), // 11
        combinatorial_constexpr(11, 11)  // 1
    };

    alignas(32) static constexpr float BEZIER_STEPS[12] = {-1.0f, -1.4f, -1.5f, -1.5f, -1.5f, 0.0f,
                                                           0.0f,  0.0f,  1.5f,  1.5f,  1.4f,  1.0f};

    alignas(32) static constexpr float BEZIER_HEIGHTS[12] = {0.0f, 0.0f, 0.9f, 0.9f, 0.9f, 0.9f,
                                                             0.9f, 1.1f, 1.1f, 1.1f, 0.0f, 0.0f};

    void stanceCurve(const float length, const float angle, const float* depth, const float phase, float* point) {
        float step = length * (1.0f - 2.0f * phase);
        point[0] += step * std::cos(angle);
        point[1] += step * std::sin(angle);

        if (length != 0.0f) {
            point[2] = *depth * std::cos((M_PI * (point[0] + point[1])) / (2.f * length));
        }
    }

    void bezierCurve(const float length, const float angle, const float* height, const float phase, float* point) {
        const float X_POLAR = std::cos(angle);
        const float Z_POLAR = std::sin(angle);

        float phase_power = 1.0f;
        float inv_phase_power = std::pow(1.0f - phase, 11);
        const float one_minus_phase = 1.0f - phase;

        for (int i = 0; i < 12; i++) {
            float b = COMBINATORIAL_VALUES[i] * phase_power * inv_phase_power;
            point[0] += b * BEZIER_STEPS[i] * length * X_POLAR;
            point[1] += b * BEZIER_STEPS[i] * length * Z_POLAR;
            point[2] += b * BEZIER_HEIGHTS[i] * *height;

            phase_power *= phase;
            inv_phase_power /= one_minus_phase;
        }
    }

    float yawArc(const float default_foot_pos[4], const float* current_pos) {
        const float foot_mag = std::hypot(default_foot_pos[0], default_foot_pos[1]);
        const float foot_dir = std::atan2(default_foot_pos[1], default_foot_pos[0]);
        const float offsets[] = {current_pos[0] - default_foot_pos[0], current_pos[2] - default_foot_pos[2],
                                 current_pos[1] - default_foot_pos[1]};
        const float offset_mag = std::hypot(offsets[0], offsets[1]);
        const float offset_mod = std::atan2(offset_mag, foot_mag);

        return M_PI_2 + foot_dir + offset_mod;
    }

    void advancePhase(float dt, float velocity) { phase = std::fmod(phase + dt * velocity, 1.0f); }

    std::tuple<float, std::function<void(const float, const float, const float*, const float, float*)>, float>
    phaseParams(float phase, float standFrac, float depth, float height) {
        if (phase < standFrac) {
            return {phase / standFrac,
                    [this](const float l, const float a, const float* d, const float p, float* pt) {
                        this->stanceCurve(l, a, d, p, pt);
                    },
                    -depth};
        }
        return {(phase - standFrac) / (1 - standFrac),
                [this](const float l, const float a, const float* h, const float p, float* pt) {
                    this->bezierCurve(l, a, h, p, pt);
                },
                height};
    }

    bool positionsMatch(const float lhs[6][4], const float rhs[6][4], float epsilon = 0.5f) const {
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (std::fabs(lhs[i][j] - rhs[i][j]) > epsilon) return false;
            }
        }
        return true;
    }

    void updateDefaultPositionTarget(int footIndex, float swingProgress) {
        for (int j = 0; j < 4; ++j) {
            defaultPosition[footIndex][j] =
                lerp(swingStartPosition[footIndex][j], targetDefaultPosition[footIndex][j], swingProgress);
        }
    }

    static constexpr std::array<float, 5> GAIT_PHASE_COUNT = {
        2.0f, // TRI_GATE
        1.0f, // BI_GATE
        6.0f, // WAVE
        6.0f, // RIPPLE
        6.0f, // AUTO
    };

    float gaitPhaseScale(GaitType gaitType) const {
        const float base = GAIT_PHASE_COUNT[static_cast<size_t>(GaitType::RIPPLE)];
        auto index = static_cast<size_t>(gaitType);
        if (gaitType == GaitType::AUTO) index = static_cast<size_t>(GaitType::RIPPLE);
        const float phases = std::max(GAIT_PHASE_COUNT[index], 1.0f);
        return base / phases;
    }

    float gaitMaxBodyVelocity(GaitType gaitType) const {
        const float standFrac = gaitStandFraction(gaitType);
        const float swingFrac = std::max(1.0f - standFrac, 0.05f);
        const float phaseScale = gaitPhaseScale(gaitType);
        return FOOT_PATH_MAX_SPEED_MM_S * (swingFrac / standFrac) * BODY_SPEED_SAFETY * phaseScale;
    }

    float gaitStandFraction(GaitType gaitType) const {
        switch (gaitType) {
            case GaitType::TRI_GATE: return 3.1f / 6.0f;
            case GaitType::BI_GATE: return 2.1f / 6.0f;
            case GaitType::WAVE: return 5.0f / 6.0f;
            case GaitType::RIPPLE:
            case GaitType::AUTO: return 5.0f / 6.0f;
            default: return default_stand_frac;
        }
    }

    void gaitOffsets(GaitType gaitType, float (&offsets)[6]) const {
        switch (gaitType) {
            case GaitType::TRI_GATE:
                offsets[0] = 0.0f;
                offsets[1] = 0.52f;
                offsets[2] = 0.08f;
                offsets[3] = 0.58f;
                offsets[4] = 0.16f;
                offsets[5] = 0.66f;
                break;
            case GaitType::BI_GATE:
                offsets[0] = 0.0f;
                offsets[1] = 1.0f / 3.0f;
                offsets[2] = 2.0f / 3.0f;
                offsets[3] = 2.0f / 3.0f;
                offsets[4] = 1.0f / 3.0f;
                offsets[5] = 0.0f;
                break;
            case GaitType::WAVE:
                offsets[0] = 0.0f;
                offsets[1] = 1.0f / 6.0f;
                offsets[2] = 2.0f / 6.0f;
                offsets[3] = 5.0f / 6.0f;
                offsets[4] = 4.0f / 6.0f;
                offsets[5] = 3.0f / 6.0f;
                break;
            case GaitType::RIPPLE:
            case GaitType::AUTO:
                offsets[0] = 0.0f;
                offsets[1] = 4.0f / 6.0f;
                offsets[2] = 2.0f / 6.0f;
                offsets[3] = 1.0f / 6.0f;
                offsets[4] = 5.0f / 6.0f;
                offsets[5] = 3.0f / 6.0f;
                break;
        }
    }

    GaitType selectAutoGait(float normalizedCommand) const {
        if (normalizedCommand < AUTO_RIPPLE_MAX_INPUT) return GaitType::RIPPLE;
        if (normalizedCommand < AUTO_TRI_MAX_INPUT) return GaitType::TRI_GATE;
        return GaitType::BI_GATE;
    }

    float desiredBodyVelocity(float normalizedCommand, GaitType selectedGait) const {
        normalizedCommand = CLIP(normalizedCommand, 0.0f, 1.0f);

        if (selectedGait == GaitType::AUTO) selectedGait = selectAutoGait(normalizedCommand);

        const float rippleMax = gaitMaxBodyVelocity(GaitType::RIPPLE);
        const float triMax = gaitMaxBodyVelocity(GaitType::TRI_GATE);
        const float biMax = gaitMaxBodyVelocity(GaitType::BI_GATE);

        if (selectedGait == GaitType::RIPPLE) {
            const float band = CLIP(normalizedCommand / AUTO_RIPPLE_MAX_INPUT, 0.0f, 1.0f);
            return rippleMax * band * 0.75f;
        }

        if (selectedGait == GaitType::TRI_GATE) {
            const float minBand = AUTO_RIPPLE_MAX_INPUT;
            const float maxBand = AUTO_TRI_MAX_INPUT;
            if (normalizedCommand < minBand) return rippleMax * 0.75f;
            const float band = CLIP((normalizedCommand - minBand) / (maxBand - minBand), 0.0f, 1.0f);
            return lerp(rippleMax * 0.75f, triMax * 0.8f, band);
        }

        if (selectedGait == GaitType::BI_GATE) {
            const float startBand = AUTO_TRI_MAX_INPUT;
            if (normalizedCommand < startBand) return triMax * 0.8f;
            const float band = CLIP((normalizedCommand - startBand) / (1.0f - startBand), 0.0f, 1.0f);
            return lerp(triMax * 0.8f, biMax, band);
        }

        return std::min(normalizedCommand * biMax, gaitMaxBodyVelocity(selectedGait));
    }

  public:
    GaitController() {}

    void setDefaultFootTarget(const float target[6][4]) { COPY_2D_ARRAY_6x4(targetDefaultPosition, target); }

    void snapDefaultFootTarget(const float target[6][4]) {
        COPY_2D_ARRAY_6x4(defaultPosition, target);
        COPY_2D_ARRAY_6x4(targetDefaultPosition, target);
        COPY_2D_ARRAY_6x4(swingStartPosition, target);
        for (bool &footSwinging : footWasSwinging) footSwinging = false;
        phase = 0.0f;
    }

    bool hasPendingStanceChange() const { return !positionsMatch(defaultPosition, targetDefaultPosition); }

    void setGait(gait_state_t& gait) {
        gaitOffsets(gait.gait_type, gait.offset);
        gait.stand_frac = gaitStandFraction(gait.gait_type);
    }

    void step(gait_state_t& gait, BodyStateMsg& body, float dt) {
        const float step_x = gait.step_x;
        const float step_z = gait.step_z;
        const float angle = gait.step_angle;
        const bool isMoving = std::fabs(step_x) >= 2 || std::fabs(step_z) >= 2 || angle;
        const bool isRepositioning = !isMoving && hasPendingStanceChange();
        const float translationMagnitude = std::hypot(step_x, step_z);
        const float translationCommand = CLIP(translationMagnitude / MAX_COMMAND_TRANSLATION_MM, 0.0f, 1.0f);
        const float turnCommand = CLIP(std::fabs(angle) / MAX_COMMAND_TURN_RAD, 0.0f, 1.0f);
        const float baseCommand = std::max(translationCommand, turnCommand);
        const float speedLimit = CLIP(gait.step_speed, 0.0f, 1.0f);
        float normalizedCommand = baseCommand;
        if (gait.gait_type == GaitType::AUTO) {
            normalizedCommand = lerp(autoCommandState, baseCommand, 0.12f);
            autoCommandState = normalizedCommand;
        } else {
            normalizedCommand = baseCommand * speedLimit;
            autoCommandState = normalizedCommand;
        }
        const GaitType activeGait = gait.gait_type == GaitType::AUTO ? selectAutoGait(normalizedCommand) : gait.gait_type;
        float activeOffset[6];
        gaitOffsets(activeGait, activeOffset);
        const float activeStandFrac = gaitStandFraction(activeGait);
        const float activeStepHeight = gait.step_height;

        if (!isMoving && !isRepositioning) {
            for (int i = 0; i < 6; i++) {
                for (int j = 0; j < 4; j++) {
                    body.feet[i][j] += (defaultPosition[i][j] - body.feet[i][j]) * dt * 10.0f;
                }
            }
            phase = 0;
            return;
        }

        const float length = std::hypot(step_x, step_z) * (gait.step_x < 0 ? -1 : 1);
        const float turnAmplitude = std::atan2(step_z, length) * 2;
        const float effectiveStepLength =
            CLIP(std::max({translationMagnitude, std::fabs(angle) * TURN_EQUIVALENT_STEP_MM, MIN_STEP_LENGTH_MM}),
                 MIN_STEP_LENGTH_MM, MAX_STEP_LENGTH_MM);
        const float desiredVelocity = gait.gait_type == GaitType::AUTO
                                          ? desiredBodyVelocity(normalizedCommand, activeGait)
                                          : std::min(normalizedCommand * gaitMaxBodyVelocity(GaitType::BI_GATE),
                                                     gaitMaxBodyVelocity(activeGait));
        const float speed = isRepositioning ? REPOSITION_PHASE_SPEED : (desiredVelocity * activeStandFrac / effectiveStepLength);

        advancePhase(dt, speed);

        float newFeet[6][4];
        COPY_2D_ARRAY_6x4(newFeet, defaultPosition);

        for (int i = 0; i < 6; i++) {
            const float* currentFoot = body.feet[i];
            const float phase = std::fmod(this->phase + activeOffset[i], 1.0f);
            const bool isSwinging = phase >= activeStandFrac;

            if (isSwinging && !footWasSwinging[i]) {
                for (int j = 0; j < 4; ++j) {
                    swingStartPosition[i][j] = defaultPosition[i][j];
                }
            }
            footWasSwinging[i] = isSwinging;

            if (isSwinging) {
                const float swingProgress = (phase - activeStandFrac) / (1.0f - activeStandFrac);
                updateDefaultPositionTarget(i, swingProgress);
            }

            auto [phNorm, curveFn, amp] = phaseParams(phase, activeStandFrac, gait.step_depth, activeStepHeight);

            float deltaPos[3] = {0, 0, 0};
            curveFn(length / 2, turnAmplitude, &amp, phNorm, deltaPos);

            float deltaRot[3] = {0, 0, 0};
            curveFn((angle * 180) / M_PI, yawArc(defaultPosition[i], currentFoot), &amp, phNorm, deltaRot);

            for (int j = 0; j < 3; j++) {
                newFeet[i][j] = defaultPosition[i][j] + deltaPos[j] + deltaRot[j];
            }
            newFeet[i][3] = 1;
        }

        body.updateFeet(newFeet);
    }
};
