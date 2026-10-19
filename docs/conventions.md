# Conventions

The one place the body frame, the leg numbering and the command signs are stated.
Every platform's gait is tested against the fixture `platform_shared/sign_convention.json`, so a mirrored axis in any one copy fails a test.

## Body frame

The body frame is right-handed: x lateral with +x to the robot's right, y forward, z up.
The robot faces +y.
Yaw is positive counter-clockwise seen from above, the right-hand rotation about +z that turns +x towards +y.
Lengths are millimetres in the firmware and the app, metres in the MuJoCo simulation.

This differs from ROS REP-103 (x forward, y left) on purpose.
Swapping it is not a rename: it reaches the leg geometry, the mounting angles, the yaw term in `generateFeet`, the MuJoCo model, the three.js scene, the searched gait gains in `gait_tuned.h` and the observation layout of every trained policy.

The gait command fields follow the frame: `step_x` is lateral stride, `step_y` forward stride, `step_angle` yaw per stance in radians.
The simulation's velocity command is `(vx, vy, yaw_rate)` with `vx` forward along body +y and `vy` lateral along body +x (`simulation/src/envs/hexapod_mj_env.py`).

## Leg numbering

Leg `i` drives servos `3i` (coxa), `3i + 1` (femur) and `3i + 2` (tibia).
Legs 0-2 are the right side, front to rear; legs 3-5 the left side, front to rear.

```
                 +y (forward)
                      ^
     3 left front     |     0 right front
            o---------+---------o
                      |
  4 left middle o-----+-----o 1 right middle  --> +x (right)
                      |
            o---------+---------o
     5 left rear      |     2 right rear
```

Derivation: `hexapodConfig` in `firmware/include/kinematics.h` mounts legs 0-2 at positive `mountX` and legs 3-5 at negative `mountX`, and orders each side by `mountY` from +74.82 mm through 0 to -74.82 mm.
The gait's resting feet in `firmware/include/gait.h` (`defaultPosition`, e.g. `{122, 152, -66}` for leg 0) lie on the same side and in the same order.
The native test checks both the gait's resting feet and the mounts against the fixture's leg sides; the simulation test checks its resting feet and mounts, and the app test checks the resting stance its 3D view walks on.
With +x to the right, positive `mountX` is the right side; the app's `LEG_NAMES` (`app/src/lib/calibration/leg-pose.ts`) name the legs the same way.

## Command signs

`GaitController::generateFeet` gives every foot the stroke `(step_x - step_angle * ry, step_y + step_angle * rx)`, where `(rx, ry)` is the foot's resting position: the commanded translation plus the yaw rate crossed with the foot's radius.
`stanceCurve` moves a planted foot from `+stroke / 2` to `-stroke / 2` along that stroke, so during stance each foot travels against its stroke, and the body, which the planted feet push, travels with it.

| Command | Gait command | Stance foot moves | Body moves |
| --- | --- | --- | --- |
| forward | `step_y > 0` | -y, every leg | +y |
| right strafe | `step_x > 0` | -x, every leg | +x |
| counter-clockwise turn | `step_angle > 0` | -y on legs 0-2, +y on legs 3-5; +x on the front legs, -x on the rear legs, none on the middle legs | yaw +, no translation |

The turn row is the cross product: a foot at `(rx, ry)` moves by `-step_angle * (-ry, rx)`, which is clockwise about the body centre, so the body turns counter-clockwise.

The fixture stores, for each of the three commands, the stick input, the gait command, the sign of each leg's mean stance displacement, and the signs of the body velocity and yaw rate those planted feet imply.
It is asserted on the firmware gait (`firmware/test/test_sign_convention`), the simulation's port (`simulation/test_sign_convention.py`) and the app, which runs the firmware's code as WebAssembly and is checked from the stick on (`app/tests/unit/sign-convention.spec.ts`).

## Stick signs

The sticks are read by an operator standing behind the robot, facing the way it faces: a stick pushed right moves the robot to its right.
The stick-to-gait mapping is a separate layer, `MotionService::handleCommand` in `firmware/include/motion.h`, with a port in `simulation/src/robot/firmware_gait.py` (`command_to_walk_gait`); the app runs the firmware's own through the virtual robot.
Its classical WALK branch takes `classicWalkTarget` (`firmware/include/gait_speed.h`), which gives `step_x` the sign of `lx`, `step_y` the sign of `ly` and `step_angle` the sign of `-rx`, scaled to the gait's speed envelope.
The TUNED branch uses the same signs through the velocity command `(ly, lx, -rx)`, forward, right and counter-clockwise, and `WALK_NN` converts the sticks the same way into the command its policy was trained on (`PolicyRunner::setCommand`, `firmware/include/policy_runner.h`).
Under the convention above that means +ly walks forward, +lx strafes right and +rx turns clockwise.
The app and a gamepad send their sticks unchanged, with +x to the right and +y up.
The ESP-NOW handheld's firmware is not in this repo: it applies a per-axis inversion before sending (`firmware/include/communication/controller_packet.h`), and the robot keeps their signs (`firmware/include/communication/handheld_input.h`).
Whether it sends +x for a stick pushed right is unverified; a bench check with the handheld would settle it, and if it was inverted to suit the previous mapping, it now strafes and turns the wrong way.
The simulation measured these signs (`simulation/src/hexapod/speed_model.py`).
The fixture records the mapping, so right strafe is the stick `lx = 0.5` and the counter-clockwise turn `rx = -0.5`.
The native test checks it on `classicWalkTarget`, the Python test on the port, and the app test from the stick on through the virtual robot.

The goal controller (`firmware/include/goal_controller.h`) reasons in the body frame and the odometry's counter-clockwise yaw, and turns the walk it wants into sticks by this mapping.
The Python facade turns REP-103 velocities into sticks through the speed model's measured table, which carries the signs.

Until 2026-10-19 the lateral and turn signs were the mirror image: +lx strafed left, +rx turned counter-clockwise, and in STAND +lx shifted the body left and +rx lowered its left side.
On the robot wired as its own mirror image, before the fix under "Floor confirmation", that mapping would strafe and turn the robot to the right for a stick pushed right, which is presumably why it went unnoticed; this is inferred from the wiring, not observed.
Telemetry recorded before the change holds sticks of the old mapping, and `simulation/score_track.py` reads a recording by the current one, so it refuses one whose header names no firmware version or one older than 0.1.1.
Version 0.1.1 is not a clean boundary: the 0.1.1 release has the old mapping, and builds after it report 0.1.1 on either mapping, so a 0.1.1 recording is scored and the next release's version is the first that tells them apart (`STICK_MAPPING_VERSION` in `simulation/score_track.py`).

## Stand stick signs

In STAND the sticks pose the body over its planted feet instead of walking it, in the same `MotionService::handleCommand`: `xm = -50 lx`, `ym = -50 ly` (mm), `phi = -0.254 rx` and `omega = 0.254 ry` (rad).
`get_transformation_matrix` (`firmware/include/kinematics.h`) maps the feet into the body, so the body moves by the inverse of that pose: a positive `xm` moves it -x, a positive `ym` moves it -y, a positive `phi` turns it about +y by `-phi`, and a positive `omega` turns it about +x by `-omega`.

| Stick | Body shift over its feet | Body tilt |
| --- | --- | --- |
| left stick right, `lx > 0` | +x, toward its right | none |
| left stick up, `ly > 0` | +y, forward | none |
| right stick right, `rx > 0` | none | right side down |
| right stick up, `ry > 0` | none | nose down |

The lateral sign is the walk's: a stick pushed right strafes the robot to its right and shifts a standing body to its right.
A positive `phi` lowers the left side, which the self-levelling confirms: the measured tilt feeds `phi` too (`levelAttitude`, `firmware/include/self_level.h`), so if a positive `phi` lowered the right side, the levelling would push a low right side further down instead of reaching down with it, as the floor check of 2026-10-16 found it does.
The stick therefore feeds `phi` negated.
A floor check on 2026-10-18, under the previous mapping, reported the left stick pushed right shifting the body right and the right stick pushed right putting the right side down.
By the derivation above the previous mapping shifted the body to its left and lowered its left side for those sticks, so the report is read as made from in front of the robot.
That reading is an inference consistent with the 2026-10-16 floor check and the self-levelling argument, not a re-observation; a floor check of the current firmware would confirm it.
Self-levelling also takes back part of a commanded tilt on flat ground, since it adds the measured tilt to the command: the simulation settles at 3.6 deg for a half stick that commands 7.3 deg.

The fixture's `stand` rows hold each stick from a settled stand and store the signs of the body's shift and tilt.
The native test (`firmware/test/test_sign_convention`) and the app test (`app/tests/unit/sign-convention.spec.ts`) read them from the feet the firmware's kinematics solves the legs for, through the virtual robot; the simulation test (`simulation/test_sign_convention.py`) reads them from the MuJoCo robot's base under the firmware's control loop and self-levelling.

## Floor confirmation

Floor confirmation: performed on 2026-10-16, after the wiring fix below.
The convention above is derived from the code, and the MuJoCo model agrees by construction: `simulation/src/resources/build_model.py` places leg 0 at +x +y in MuJoCo's right-handed, z-up world.
A bench check on 2026-10-16 found the robot built as the mirror image of that convention: with its left side lifted, its right legs reached down while the self-levelling extended legs 3-5.
Its servos 0-8 were wired to the board at 0x40, the left side: on the servo page, servo 0 moved the left front coxa.
Raising servo 0's pulse swung that foot toward the rear, a counter-clockwise turn seen from above, so a rising pulse turns a coxa counter-clockwise on either side.
Servos 0-8 now drive the board at 0x41, the right side, with every coxa direction reversed (`servo_controller.h`, `servo_factory.h`; `firmware/test/test_servo_map`).
With the fix flashed and the calibration moved with its servos, servo 0 moves the right front coxa, raising its pulse swings that foot forward, and the self-levelling reaches down with whichever side is low.
On the floor a forward stick walks the robot toward its front, a turn command with `step_angle > 0` turns it counter-clockwise, `step_x > 0` strafes it right, and the app's 3D view moves the same way.

## IMU angles

The MPU6050 DMP's roll agrees with the body frame, while its pitch and yaw are mirrored (`firmware/include/imu_frame.h`); its gravity vector and gyro agree.
Measured on the robot on 2026-10-16: right side down 17.7 deg read pitch -17.7 deg and world up (-0.30, 0, 0.95), nose down 12 deg read roll -12.0 deg, and a counter-clockwise turn read a falling yaw with a positive gyro z.
Self-levelling and the policy's observation turn the DMP's angles into the body frame with those signs, and the simulations feed the firmware the DMP's angles, not their own.

## Compass heading

The heading is `atan2(y, x)` of the compass reading in the chip frame, plus the declination, in degrees in [0, 360).
That is a compass heading, clockwise from north, only if the chip's +x points forward and +y to the left (REP-103 axes).
It therefore has the opposite sign to the body yaw above, which is counter-clockwise: comparing the two needs a negation, and the rotation of the chip's mounting relative to the body frame.
That mounting rotation is not measured; the heading has not been checked against the DMP yaw on the robot.
The hard-iron offset (`PeripheralSettings.mag_offset_x/y/z`, fitted by `simulation/calibrate_compass.py`) is subtracted in the chip frame before the heading is taken.
