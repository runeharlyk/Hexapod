#include <spot.h>
#include <WiFi.h>
#include <Wire.h>

DRAM_ATTR Spot spot;
#include <Adafruit_PWMServoDriver.h>

#include "config.h"
#include "motion.h"

Adafruit_PWMServoDriver left_pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver right_pwm = Adafruit_PWMServoDriver(0x41);

MotionMode current_motion = MotionMode::Mode_Standby;
MotionMode next_motion = MotionMode::Mode_Standby;

void posture_calibration();
void boot_up_motion(int lut_size, int lut[][6][3]);
void exec_motion(int lut_size, int lut[][6][3]);
void exec_transition(int start_pos[][6][3], int start_pos_idx, int end_pos[][6][3], int end_pos_idx);
void test_wiggle();

void setup() {
    Serial.begin(115200);

    spot.initialize();

    Serial.println("Starting servos");

    left_pwm.begin();
    left_pwm.setPWMFreq(50);

    right_pwm.begin();
    right_pwm.setPWMFreq(50);

    ESP_LOGI("main", "Setup complete now runing tsk");
    delay(1000);

    posture_calibration();
    // test_wiggle();
    // boot_up_motion
    // posture_calibration();

    boot_up_motion(lut_standup_length, lut_standup);

    delay(3000);
}

void loop() {
    exec_motion(lut_twist_length, lut_twist);
    // exec_motion(lut_walk_180_length, lut_walk_180);
    // if (next_motion == MotionMode::Mode_Walk_0) {
    //     exec_motion(lut_walk_0_length, lut_walk_0);
    // } else if (next_motion == MotionMode::Mode_Walk_180) {
    //     exec_motion(lut_walk_180_length, lut_walk_180);
    // } else if (next_motion == MotionMode::Mode_Walk_R45) {
    //     exec_motion(lut_walk_r45_length, lut_walk_r45);
    // } else if (next_motion == MotionMode::Mode_Walk_R90) {
    //     exec_motion(lut_walk_r90_length, lut_walk_r90);
    // } else if (next_motion == MotionMode::Mode_Walk_R135) {
    //     exec_motion(lut_walk_r135_length, lut_walk_r135);
    // } else if (next_motion == MotionMode::Mode_Walk_L45) {
    //     exec_motion(lut_walk_l45_length, lut_walk_l45);
    // } else if (next_motion == MotionMode::Mode_Walk_L90) {
    //     exec_motion(lut_walk_l90_length, lut_walk_l90);
    // } else if (next_motion == MotionMode::Mode_Walk_L135) {
    //     exec_motion(lut_walk_l135_length, lut_walk_l135);
    // } else if (next_motion == MotionMode::Mode_Fast_Forward) {
    //     exec_motion(lut_fast_forward_length, lut_fast_forward);
    // } else if (next_motion == MotionMode::Mode_Fast_Backward) {
    //     exec_motion(lut_fast_backward_length, lut_fast_backward);
    // } else if (next_motion == MotionMode::Mode_Turn_Left) {
    //     exec_motion(lut_turn_left_length, lut_turn_left);
    // } else if (next_motion == MotionMode::Mode_Turn_Right) {
    //     exec_motion(lut_turn_right_length, lut_turn_right);
    // } else if (next_motion == MotionMode::Mode_Climb_Forward) {
    //     exec_motion(lut_climb_forward_length, lut_climb_forward);
    // } else if (next_motion == MotionMode::Mode_Climb_Backward) {
    //     exec_motion(lut_climb_backward_length, lut_climb_backward);
    // } else if (next_motion == MotionMode::Mode_Rotate_X) {
    //     exec_motion(lut_rotate_x_length, lut_rotate_x);
    // } else if (next_motion == MotionMode::Mode_Rotate_Y) {
    //     exec_motion(lut_rotate_y_length, lut_rotate_y);
    // } else if (next_motion == MotionMode::Mode_Rotate_Z) {
    //     exec_motion(lut_rotate_z_length, lut_rotate_z);
    // } else if (next_motion == MotionMode::Mode_Twist) {
    //     exec_motion(lut_twist_length, lut_twist);
    // } else {
    //     exec_motion(lut_standby_length, lut_standby);
    // }
}

/**
   @brief Calibrates the posture of the hexapod robot.

   This function sets the PWM values for each joint of the hexapod's legs to a
   neutral position, defined by SERVOMID and the offset values stored in
   right_offset_ticks and left_offset_ticks. This helps ensure that the robot
   starts in a balanced and stable posture.
*/
void posture_calibration() {
    for (int leg_idx = 0; leg_idx < 3; leg_idx++) {
        for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
            right_pwm.setPWM(legs_order[leg_idx][joint_idx], 0, SERVOMID + right_offset_ticks[leg_idx][joint_idx]);
            left_pwm.setPWM(legs_order[leg_idx][joint_idx], 0, SERVOMID + left_offset_ticks[leg_idx][joint_idx]);
        }
    }
}

void boot_up_motion(int lut_size, int lut[][6][3]) {
    for (int leg_idx = 0; leg_idx < 3; leg_idx++) {
        for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
            right_pwm.setPWM(legs_order[leg_idx][joint_idx], 0,
                             lut[0][leg_idx][joint_idx] + right_offset_ticks[leg_idx][joint_idx]);
            delay(50);
            left_pwm.setPWM(legs_order[leg_idx][joint_idx], 0,
                            lut[0][leg_idx + 3][joint_idx] + left_offset_ticks[leg_idx][joint_idx]);
            delay(50);
        }
    }

    for (int lut_idx = 0; lut_idx < lut_size; lut_idx++) {
        for (int leg_idx = 0; leg_idx < 3; leg_idx++) {
            for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
                right_pwm.setPWM(legs_order[leg_idx][joint_idx], 0,
                                 lut[lut_idx][leg_idx][joint_idx] + right_offset_ticks[leg_idx][joint_idx]);
                left_pwm.setPWM(legs_order[leg_idx][joint_idx], 0,
                                lut[lut_idx][leg_idx + 3][joint_idx] + left_offset_ticks[leg_idx][joint_idx]);
            }
        }
        delay(DELAY_MS);
    }
}

/**
   @brief Executes a motion sequence defined by a lookup table (LUT).

   This function iterates through the provided LUT, setting the PWM values for
   each joint of the hexapod's legs. It handles transitions between different
   motion modes and ensures smooth movement.

   @param lut_size The size of the LUT.
   @param lut The LUT containing the PWM values for each joint at each step of
   the motion.
*/
void exec_motion(int lut_size, int lut[][6][3]) {
    int mid_step = (int)(lut_size / 2);
    if (current_motion == MotionMode::Mode_Standby) {
        exec_transition(lut_standby, 0, lut, 0);
    }
    current_motion = next_motion;

    for (int lut_idx = 0; lut_idx < lut_size; lut_idx++) {
        for (int leg_idx = 0; leg_idx < 3; leg_idx++) {
            for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
                int right_value = lut[lut_idx][leg_idx][joint_idx];
                int left_value = lut[lut_idx][leg_idx + 3][joint_idx];

                // For joint_idx 1 (second joint), invert the offset from SERVOMID
                if (leg_idx == 1 && joint_idx != 0) {
                    right_value = SERVOMID - (right_value - SERVOMID);
                    left_value = SERVOMID - (left_value - SERVOMID);
                }

                right_pwm.setPWM(legs_order[leg_idx][joint_idx], 0,
                                 right_value + right_offset_ticks[leg_idx][joint_idx]);
                left_pwm.setPWM(legs_order[leg_idx][joint_idx], 0, left_value + left_offset_ticks[leg_idx][joint_idx]);
            }
        }

        if (mid_step > 0) {
            if (lut_idx % mid_step == 0 && current_motion != next_motion) {
                exec_transition(lut, lut_idx, lut_standby, 0);
                delay(DELAY_MS);
                break;
            }
        }
        delay(DELAY_MS);
    }
}

/**
   @brief Executes a smooth transition between two motion positions.

   This function takes two motion positions (start_pos and end_pos) and their
   respective indices in their respective LUTs. It calculates the difference
   between the start and end positions for each joint and determines the number
   of steps required for a smooth transition. The function then iterates through
   these steps, adjusting the PWM values for each joint to gradually move from
   the start position to the end position.

   @param start_pos The starting position LUT.
   @param start_pos_idx The index of the starting position in the start_pos LUT.
   @param end_pos The ending position LUT.
   @param end_pos_idx The index of the ending position in the end_pos LUT.
*/
void exec_transition(int start_pos[][6][3], int start_pos_idx, int end_pos[][6][3], int end_pos_idx) {
    int tick_step = 6;
    int max_step = 0;
    int signed_ticks[6][3];

    int current_pos[6][3];
    int diff;

    for (int leg_idx = 0; leg_idx < 6; leg_idx++) {
        for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
            diff = end_pos[end_pos_idx][leg_idx][joint_idx] - start_pos[start_pos_idx][leg_idx][joint_idx];
            current_pos[leg_idx][joint_idx] = start_pos[start_pos_idx][leg_idx][joint_idx];
            if (diff < 0) {
                signed_ticks[leg_idx][joint_idx] = -tick_step;
            } else {
                signed_ticks[leg_idx][joint_idx] = tick_step;
            }
            max_step = max(max_step, abs(diff));
        }
    }
    max_step = ceil(max_step / tick_step);
    for (int step_idx = 0; step_idx < max_step; step_idx++) {
        for (int leg_idx = 0; leg_idx < 3; leg_idx++) {
            for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
                if (abs(current_pos[leg_idx][joint_idx] - end_pos[end_pos_idx][leg_idx][joint_idx]) > tick_step) {
                    current_pos[leg_idx][joint_idx] =
                        current_pos[leg_idx][joint_idx] + signed_ticks[leg_idx][joint_idx];
                } else {
                    current_pos[leg_idx][joint_idx] = end_pos[end_pos_idx][leg_idx][joint_idx];
                }

                if (abs(current_pos[leg_idx + 3][joint_idx] - end_pos[end_pos_idx][leg_idx + 3][joint_idx]) >
                    tick_step) {
                    current_pos[leg_idx + 3][joint_idx] =
                        current_pos[leg_idx + 3][joint_idx] + signed_ticks[leg_idx + 3][joint_idx];
                } else {
                    current_pos[leg_idx + 3][joint_idx] = end_pos[end_pos_idx][leg_idx + 3][joint_idx];
                }

                right_pwm.setPWM(legs_order[leg_idx][joint_idx], 0,
                                 current_pos[leg_idx][joint_idx] + right_offset_ticks[leg_idx][joint_idx]);
                left_pwm.setPWM(legs_order[leg_idx][joint_idx], 0,
                                current_pos[leg_idx + 3][joint_idx] + left_offset_ticks[leg_idx][joint_idx]);
            }
        }
    }
}

void test_wiggle() {
    const int ANGLE_10_DEG = 41; // ~10 degrees in servo ticks (4.096 ticks per degree)

    // Test each leg, one at a time
    for (int leg_idx = 0; leg_idx < 3; leg_idx++) {
        Serial.println("Testing leg " + String(leg_idx));
        // Test right leg
        for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
            // Move joint positive 10 degrees
            right_pwm.setPWM(legs_order[leg_idx][joint_idx], 0,
                             SERVOMID + right_offset_ticks[leg_idx][joint_idx] + ANGLE_10_DEG);
            delay(250);
            // Move joint negative 10 degrees
            right_pwm.setPWM(legs_order[leg_idx][joint_idx], 0,
                             SERVOMID + right_offset_ticks[leg_idx][joint_idx] - ANGLE_10_DEG);
            delay(250);
            // Return to center
            right_pwm.setPWM(legs_order[leg_idx][joint_idx], 0, SERVOMID + right_offset_ticks[leg_idx][joint_idx]);
            delay(250);
            Serial.println("Right leg joint " + String(joint_idx) + " moved to " +
                           String(right_offset_ticks[leg_idx][joint_idx]));
        }

        // Test left leg
        for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
            // Move joint positive 10 degrees
            left_pwm.setPWM(legs_order[leg_idx][joint_idx], 0,
                            SERVOMID + left_offset_ticks[leg_idx][joint_idx] + ANGLE_10_DEG);
            delay(250);
            // Move joint negative 10 degrees
            left_pwm.setPWM(legs_order[leg_idx][joint_idx], 0,
                            SERVOMID + left_offset_ticks[leg_idx][joint_idx] - ANGLE_10_DEG);
            delay(250);
            // Return to center
            left_pwm.setPWM(legs_order[leg_idx][joint_idx], 0, SERVOMID + left_offset_ticks[leg_idx][joint_idx]);
            delay(250);
            Serial.println("Left leg joint " + String(joint_idx) + " moved to " +
                           String(left_offset_ticks[leg_idx][joint_idx]));
        }
        delay(500); // Pause between testing each leg pair
    }
}
