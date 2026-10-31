# Adjacent-leg clearance over the command envelope

Can two neighbouring legs touch anywhere in the stick envelope?
Full strafe plus full yaw brings neighbouring legs closest, and the simulation cannot answer the question, so `leg_clearance.py` answers it geometrically.

## The MuJoCo model has no leg-leg contacts

`build_model.py` gives every leg link capsule contype 4 / conaffinity 1 and every foot sphere contype 2 / conaffinity 1.
MuJoCo pairs two geoms only when `(contype_a & conaffinity_b) || (contype_b & conaffinity_a)` is non-zero, and `4 & 1`, `2 & 1` are both zero, so no link or foot of one leg ever collides with another leg's.
The model declares no explicit `<contact><pair>` either.
Loading `src/resources/model.xml` and `src/resources/model_terrain.xml` confirms it: of the 24 leg geoms, 0 pairs are collidable, with `npair = 0` and `nexclude = 0`.
A gait that drove two legs through each other would therefore run in the sim without complaint.

## Method

For each gait, every stick command `(lx, ly, rx)` on a grid over [-1, 1]^3 goes through `command_to_walk_gait`, the firmware gait port generates the feet at evenly spaced phases of one cycle, and the port's IK turns them into joint angles.
The command is taken as already reached and the feet as already on their resting stance, so the command ramp, the stop/start easing and stance changes are outside the sweep.
`ServoController::setAngles` clips every joint to its travel before it reaches a servo, a coxa to its servo's safe pulses (about +-50.5 deg at the factory calibration, TUNED's to 31.5 deg), a femur to 90 and a tibia to 149 deg, so the default check runs on the clamped angles the robot actually reaches; the `commanded` rows use the raw IK angles for comparison.
The angles give each leg's chain mount -> hip -> knee -> tip, built with the same conventions as `Kinematics.forward_kinematics_local`; the test checks that the chain's tip reproduces the IK's target foot wherever the foot is reachable.
Neighbouring legs are the ring 0-1-2-5-4-3: the same-side pairs 0-1, 1-2, 3-4, 4-5 and the pairs across the body 0-3 and 2-5.
Each neighbouring pair's coxa, femur and tibia segments and foot point are compared by exact segment-segment distance.
Centre clearance is the least distance between centre lines; surface clearance subtracts the radii of `src/robot/clearance.py` (coxa 20.8, femur 15.5, tibia 5 mm, foot sphere 6 mm: the MuJoCo model's capsules, widened where the parts around the servos are thicker) and is negative on contact.

## Results

Default sweep: 9^3 stick commands x 40 phases, step-height stick `s1 = 0`, body-height stick `h = 0`.
Travel is the share of samples with any joint commanded beyond its travel; hits are stick commands that touch somewhere in the cycle.
The tables were measured with the coxa clipped at 31.5 deg and the MuJoCo model's capsule radii (coxa 8, femur 7, tibia 5 mm), before the coxa's travel widened to its servo's safe pulses and the radii to the parts around the servos.
The commands below are in the stick mapping before 2026-10-19, when +lx strafed left and +rx turned counter-clockwise; under today's (`docs/conventions.md`) the same motion is the stick with `lx` and `rx` negated.

| gait | travel | clamped surface (mm) | clamped hits | worst clamped command (lx, ly, rx) @ phase, legs, segments | commanded surface (mm) | commanded hits |
| ---- | ------ | -------------------- | ------------ | ---------------------------------------------------------- | ---------------------- | -------------- |
| tripod | 64.8 % | 10.4 | 0 | (+0.25, -1.00, +1.00) @ 0.400, legs 3-4, foot-foot | -13.8 | 287 of 729 |
| bipod | 76.3 % | 10.9 | 0 | (-0.25, -1.00, +1.00) @ 0.550, legs 4-5, foot-foot | -13.9 | 284 of 729 |
| wave | 46.7 % | 13.2 | 0 | (-0.75, +1.00, +1.00) @ 0.700, legs 1-2, foot-tibia | -13.9 | 164 of 729 |
| ripple | 50.8 % | 17.5 | 0 | (-1.00, -1.00, +1.00) @ 0.150, legs 4-5, foot-foot | -13.4 | 104 of 729 |
| tuned | 64.0 % | 21.7 | 0 | (+0.50, +0.75, -1.00) @ 0.550, legs 4-5, foot-foot | -12.5 | 247 of 729 |

Sweeping `s1` and `h` over {-1, 0, 1} each (TUNED rides at its searched height and ignores `h`) leaves every clamped case positive.
The least is 9.5 mm, tripod at `s1 = -1`, `h = +1`, legs 3-4 foot to foot; raising the body by `h = +1` pushes 97.5 % of those samples past travel.
A finer grid, 17^3 commands x 80 phases, moves the minimum by at most 0.2 mm: tripod 10.3 mm, tuned 21.5 mm.

## Verdict

With the servos stopped at their travel, no stick command in any gait brings two neighbouring legs into contact: the least surface clearance is 9.5 mm over the whole swept envelope, 10.4 mm at default height and lift.
The firmware now checks a pose for leg-leg contact itself (`firmware/include/leg_clearance.h`, the port of `src/robot/clearance.py`): the animation player and the joint drag move two legs within `LEG_CLEARANCE_MM`, 5 mm, of each other only to part them, and the robot's animation validation refuses a clip that brings them that close.
The margin comes from the travel clamp, though, not from the gait.
With the model's capsules, the raw IK angles collided on 104 to 287 of the 729 commands, femur against femur of neighbours such as legs 0-1 at (+0.25, -1.00, +1.00) in today's mapping, because combined forward, lateral and yaw strokes asked a coxa to swing up to 89.5 deg (tripod), far past the 31.5 deg travel it had then.
Same-side neighbours are mounted only 45 deg apart, so the travel alone does not rule contact out; it is the clamped sweep that shows the legs stay apart within it.
The coxa's travel has since widened to its servo's safe pulses, so these sweeps, measured at 31.5 deg, no longer describe the robot's walk until they are re-run at the new travel.
`test_leg_clearance.py` keeps the tripod and TUNED sweeps on a 5^3 grid at least `LEG_CLEARANCE_MM`, 5 mm, apart, so a gait change that brings the legs together fails there.
It also sweeps every classic timing in both stances at the longest stride, at the neutral posture and the posture cube's corners pulled onto the walkable boundary (`--stance`, `--stride`, `--fd`), where the least clearance is 5.9 mm (BI, legs 1-2) in the default stance and 30.3 mm in the crab.
The walkable envelope is bounded by the same check (`src/robot/gait_envelope.py`): it ends the default stance's longest forward stroke at 92 of the 508 grid postures that stride forward and its longest turn at 285 of 528, and never the crab's.

## Limits

The links are capsules sized from the meshes the app draws and the MG92B's datasheet size, not measured on the robot, and a capsule is round where the tibia blade is up to 40 mm deep in its leg's plane; the margins hold for the capsules and are unverified against the robot.
The sweep keeps the body level, as the WALK gait does; the body roll, pitch and yaw that STAND's sticks and the animations command, and the command transitions, are not covered.
Clamping a joint at travel means the foot no longer follows the planned path, so on the 47 to 76 % of samples beyond travel the robot is not walking the gait the command asked for; that is a feasibility problem in its own right, separate from clearance.

## Reproduce

```sh
uv run python leg_clearance.py                                     # default sweep, every gait
uv run python leg_clearance.py --s1 -1 0 1 --h -1 0 1              # step-height and body-height sticks
uv run python leg_clearance.py --grid 17 --phases 80 --gaits tripod tuned
uv run python leg_clearance.py --list-hits --gaits tripod          # every colliding stick command
uv run pytest -q test_leg_clearance.py
```
