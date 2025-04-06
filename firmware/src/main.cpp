#include <spot.h>
#include <WiFi.h>
#include <Wire.h>

DRAM_ATTR Spot spot;
#include <Adafruit_PWMServoDriver.h>

#include "config.h"
#include "motion_table.h"

Adafruit_PWMServoDriver left_pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver right_pwm = Adafruit_PWMServoDriver(0x41);

void IRAM_ATTR SpotControlLoopEntry(void*) {
    ESP_LOGI("main", "Setup complete now runing tsk");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;
    for (;;) {
        // spot.readSensors();
        spot.planMotion();
        CALLS_PER_SECOND(updateActuators);
        spot.updateActuators();
        spot.emitTelemetry();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
void exec_motion(int lut_size, int lut[][6][3]);

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
    exec_motion(lut_calibration_length, lut_calibration);
    delay(1000);
    exec_motion(lut_standup_length, lut_standup);
    delay(1000);
}

void loop() {
    // exec_motion(lut_twist_length, lut_twist);
    // exec_motion(lut_fast_forward_length, lut_fast_forward);
    // exec_motion(lut_twist_length, lut_twist); // Good
    exec_motion(lut_walk_0_length, lut_walk_0);
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

void exec_motion(int lut_size, int lut[][6][3]) {
    uint16_t right_pwm_values[11] = {SERVOMID};
    uint16_t left_pwm_values[11] = {SERVOMID};

    for (int lut_idx = 0; lut_idx < lut_size; lut_idx++) {
        for (int leg_idx = 0; leg_idx < 3; leg_idx++) {
            for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
                int right_value = lut[lut_idx][leg_idx][joint_idx] + right_offset_ticks[leg_idx][joint_idx];
                int left_value = lut[lut_idx][leg_idx + 3][joint_idx] + left_offset_ticks[leg_idx][joint_idx];
                int pin = legs_order[leg_idx][joint_idx];

                if (leg_idx == 1 && joint_idx != 0) {
                    right_value = SERVOMID - (right_value - SERVOMID);
                    left_value = SERVOMID - (left_value - SERVOMID);
                }

                right_pwm_values[pin] = right_value + right_offset_ticks[leg_idx][joint_idx];
                left_pwm_values[pin] = left_value + left_offset_ticks[leg_idx][joint_idx];
            }
        }
        right_pwm.setMultiplePWM(right_pwm_values, 11);
        left_pwm.setMultiplePWM(left_pwm_values, 11);

        delay(40);
    }
}
