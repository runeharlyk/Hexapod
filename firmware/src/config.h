
#ifndef CONFIG_H
#define CONFIG_H

#define SERVOMIN 102 // Minimum value, 0 deg
#define SERVOMID 307 // Middle value, 90 deg
#define SERVOMAX 512 // Maximum value, 180 deg

#define DELAY_MS 10 // Servo delay

/** Motion Mode */
enum MotionMode {
    Mode_Standby,
    Mode_Walk_0,
    Mode_Walk_180,
    Mode_Walk_R45,
    Mode_Walk_R90,
    Mode_Walk_R135,
    Mode_Walk_L45,
    Mode_Walk_L90,
    Mode_Walk_L135,
    Mode_Fast_Forward,
    Mode_Fast_Backward,
    Mode_Turn_Left,
    Mode_Turn_Right,
    Mode_Climb_Forward,
    Mode_Climb_Backward,
    Mode_Rotate_X,
    Mode_Rotate_Y,
    Mode_Rotate_Z,
    Mode_Twist,
};

// Servo connections to the PCA9685 driver
// {{leg1_join1, leg1_join2, leg1_join3},
//  {leg2_join1, leg2_join2, leg2_join3},
//  {leg3_join1, leg3_join2, leg3_join3}}
static int legs_order[3][3] = {{8, 9, 10}, {2, 3, 4}, {5, 6, 7}};

const static String _leg_table[] = {
    "front right", "right", "rear right", "rear left", "left", "front left",
};

const static String _joint_table[] = {
    "body",
    "thigh",
    "foot",
};

// Offset to correct the installation error. Offset value is the number of ticks
static int left_offset_ticks[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
static int right_offset_ticks[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

#define UDP_PORT 1234 // local port to listen on

#endif // CONFIG_H
