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
`ServoController::setAngles` clips every joint to `JOINT_LIMIT_DEG` (coxa 31.5, femur 90, tibia 149 deg) before it reaches a servo, so the default check runs on the clamped angles the robot actually reaches; the `commanded` rows use the raw IK angles for comparison.
The angles give each leg's chain mount -> hip -> knee -> tip, built with the same conventions as `Kinematics.forward_kinematics_local`; the test checks that the chain's tip reproduces the IK's target foot wherever the foot is reachable.
Neighbouring legs are the ring 0-1-2-5-4-3: the same-side pairs 0-1, 1-2, 3-4, 4-5 and the pairs across the body 0-3 and 2-5.
Each neighbouring pair's coxa, femur and tibia segments and foot point are compared by exact segment-segment distance.
Centre clearance is the least distance between centre lines; surface clearance subtracts the radii `build_model.py` gives the links (coxa 8, femur 7, tibia 5 mm, foot sphere 6 mm) and is negative on contact.

## Results

Default sweep: 9^3 stick commands x 40 phases, step-height stick `s1 = 0`, body-height stick `h = 0`.
Travel is the share of samples with any joint commanded beyond its travel; hits are stick commands that touch somewhere in the cycle.

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
No clamp task is needed for leg-leg contact.
The margin comes from the travel clamp, though, not from the gait.
The raw IK angles collide on 104 to 287 of the 729 commands, femur against femur of neighbours such as legs 0-1 at (-0.25, -1.00, -1.00), because combined forward, lateral and yaw strokes ask a coxa to swing up to 89.5 deg (tripod), far past its 31.5 deg travel.
Same-side neighbours are mounted only 45 deg apart, so the travel alone does not rule contact out; it is the clamped sweep that shows the legs stay apart within it.
Widening `JOINT_LIMIT_DEG` for the coxa, or adding a mode that bypasses `setAngles`, needs this sweep re-run before it ships.
`test_leg_clearance.py` keeps the tripod and TUNED sweeps on a 5^3 grid above 5 mm, so a gait change that brings the legs together fails there.

## Limits

The links are the capsules of the MuJoCo model, not the real servo housings and brackets, which are bulkier near the hip and knee; the 9.5 mm margin holds for the model and is unverified against the CAD.
The sweep keeps the body level, as the WALK gait does; body roll, pitch and yaw from POSE mode and the command transitions are not covered.
Clamping a joint at travel means the foot no longer follows the planned path, so on the 47 to 76 % of samples beyond travel the robot is not walking the gait the command asked for; that is a feasibility problem in its own right, separate from clearance.

## Reproduce

```sh
uv run python leg_clearance.py                                     # default sweep, every gait
uv run python leg_clearance.py --s1 -1 0 1 --h -1 0 1              # step-height and body-height sticks
uv run python leg_clearance.py --grid 17 --phases 80 --gaits tripod tuned
uv run python leg_clearance.py --list-hits --gaits tripod          # every colliding stick command
uv run pytest -q test_leg_clearance.py
```
