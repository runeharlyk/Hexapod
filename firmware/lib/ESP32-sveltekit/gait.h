#pragma once

#include <utils/math_utils.h>
#include <array>
#include <functional>
#include <kinematics.h>

enum class GaitType { TRI_GATE, BI_GATE, WAVE, RIPPLE };

static constexpr float default_offset[6] = {0, 0.5, 0, 0.5, 0, 0.5};
static constexpr float default_stand_frac = 3.1 / 6;

struct gait_state_t {
    float step_height, step_x, step_z, step_angle, step_speed, step_depth, stand_frac;
    GaitType gait_type;
    float offset[6];
};

struct ControllerCommand {
    int stop;
    float lx, ly, rx, ry, h, s, s1;
};

class GaitController {
  private:
    float phase = 0.0f;
    float defaultPosition[6][4];

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
        point[2] += step * std::sin(angle);

        if (length != 0.0f) {
            point[1] = -*depth * std::cos((M_PI * (point[0] + point[2])) / (2.f * length));
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
            point[1] += b * BEZIER_HEIGHTS[i] * *height;
            point[2] += b * BEZIER_STEPS[i] * length * Z_POLAR;

            phase_power *= phase;
            inv_phase_power /= one_minus_phase;
        }
    }

    float yawArc(const float feet_pos[4], const float* current_pos) {
        const float foot_mag = std::hypot(feet_pos[0], feet_pos[2]);
        const float foot_dir = std::atan2(feet_pos[2], feet_pos[0]);
        const float offsets[] = {current_pos[0] - feet_pos[0], current_pos[1] - feet_pos[1],
                                 current_pos[2] - feet_pos[2]};
        const float offset_mag = std::hypot(offsets[0], offsets[2]);
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

  public:
    GaitController(const float defaultPos[6][4]) { COPY_2D_ARRAY_4x4(defaultPosition, defaultPos); }

    void step(gait_state_t& gait, body_state_t& body, float dt) {
        const float step_x = gait.step_x;
        const float step_z = gait.step_z;
        const float angle = gait.step_angle;

        if (!step_x && !step_z && !angle) {
            for (int i = 0; i < 6; i++) {
                for (int j = 0; j < 4; j++) {
                    body.feet[i][j] += (defaultPosition[i][j] - body.feet[i][j]) * dt * 10;
                }
            }
            phase = 0;
            return;
        }

        advancePhase(dt, gait.step_speed);

        const float lengthRaw = std::hypot(step_x, step_z);
        const float length = step_x < 0 ? -lengthRaw : lengthRaw;
        const float turnAmplitude = std::atan2(step_z, length) * 2;

        float newFeet[6][4];
        COPY_2D_ARRAY_6x4(newFeet, defaultPosition);

        for (int i = 0; i < 6; i++) {
            const float* defaultFoot = defaultPosition[i];
            const float* currentFoot = body.feet[i];
            const float phase = std::fmod(this->phase + gait.offset[i], 1.0f);

            auto [phNorm, curveFn, amp] = phaseParams(phase, gait.stand_frac, gait.step_depth, gait.step_height);

            float deltaPos[3] = {0, 0, 0};
            curveFn(length / 2, turnAmplitude, &amp, phNorm, deltaPos);

            float deltaRot[3] = {0, 0, 0};
            curveFn((angle * 180) / M_PI, yawArc(defaultFoot, currentFoot), &amp, phNorm, deltaRot);

            for (int j = 0; j < 3; j++) {
                newFeet[i][j] = defaultFoot[j] + deltaPos[j] + deltaRot[j];
            }
            newFeet[i][3] = 1;
        }

        body.updateFeet(newFeet);
    }
};