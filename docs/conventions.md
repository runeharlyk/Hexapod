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

The stick-to-gait mapping is a separate layer, `MotionService::handleCommand` in `firmware/include/motion.h`, with a port in `simulation/src/robot/firmware_gait.py` (`command_to_walk_gait`); the app runs the firmware's own through the virtual robot.
Its classical WALK branch sets `step_x = -lx * 100`, `step_y = ly * 100` and `step_angle = rx * 0.8`; the TUNED branch uses the same signs through the velocity command `(ly, -lx, rx)`.
Under the convention above that means +ly walks forward, +lx strafes left and +rx turns counter-clockwise, so a stick pushed right on the app's joystick or a gamepad strafes and turns the robot to its left.
The simulation measured exactly this (`simulation/src/hexapod/speed_model.py`).
The fixture records the mapping as it stands, so right strafe is the stick `lx = -0.5`; the Python and app tests assert it, while the native test cannot reach `MotionService` and covers only the gait layer.

## Floor confirmation

Floor confirmation: not yet performed.
The convention above is derived from the code, and the MuJoCo model agrees by construction: `simulation/src/resources/build_model.py` places leg 0 at +x +y in MuJoCo's right-handed, z-up world.
Nothing yet shows that the physical robot's leg 0 is its right front leg, that `step_x > 0` strafes it right, or that `step_angle > 0` turns it counter-clockwise.

## Compass heading

The heading is `atan2(y, x)` of the compass reading in the chip frame, plus the declination, in degrees in [0, 360).
That is a compass heading, clockwise from north, only if the chip's +x points forward and +y to the left (REP-103 axes).
It therefore has the opposite sign to the body yaw above, which is counter-clockwise: comparing the two needs a negation, and the rotation of the chip's mounting relative to the body frame.
That mounting rotation is not measured; the heading has not been checked against the DMP yaw on the robot.
The hard-iron offset (`PeripheralSettings.mag_offset_x/y/z`, fitted by `simulation/calibrate_compass.py`) is subtracted in the chip frame before the heading is taken.
