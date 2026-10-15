# Servo model: driving torque versus holding torque

The simulated MG92B (`src/sim/mj_runtime.py`, `HexapodSim._apply_servo`) is a cascaded position -> velocity -> torque loop whose net torque depends on what the servo is doing.
Driving the joint, it nets at most its stall torque, `SERVO_STALL = 0.34` N.m, the MG92B rating at 6 V.
Against a load that would back-drive the joint, the load has to turn the gear train backwards, so a powered servo resists up to the larger holding torque `SERVO_HOLD = SERVO_K_HOLD * SERVO_STALL`.
The same holds while the servo brakes its own motion, since the joint's momentum then back-drives the gears too.

It is modelled as gear friction.
The motor actuator delivers up to the mean of stall and hold (`motor_torque_cap`, 0.595 N.m nominal), and a joint `frictionloss` of half their difference (0.255 N.m nominal, on top of the model's own 0.005) always opposes motion.
Driving nets motor - friction = stall; holding still, being back-driven or braking nets motor + friction = hold.
MuJoCo's constraint solver applies the friction implicitly, so the joint sticks and slips without the chatter a velocity-switched torque cap produces at the 2 ms physics step.
The friction constraint is stiffened (`dof_solref` time constant 2 timesteps, `dof_solimp` 0.999-0.9999): at MuJoCo's defaults it pushes with a motion the motor brakes harder than its reference (a 0.03 deg overshoot on a 20 deg step) and lets a load near either limit slip 1-3 deg in 2 s.

Being a soft constraint, the friction still lets the joint creep slowly: a load just below hold slips about 0.01 deg/s, a load just above stall that the servo is told to lift advances under 0.03 deg in 2 s, and after a step the joint creeps its last few hundredths of a degree onto the target over several seconds.

`actuator_force` reports the motor share only, so `servo_feasibility.py` torque fractions top out at (1 + k_hold) / 2 = 1.75 of stall, and the friction share is not in them.

## The placeholder

**`SERVO_K_HOLD = 2.5` is a placeholder.**
The ratio of holding to stall torque is an unverified estimate of 2 to 3; no measurement backs the value 2.5.
Domain randomization (`HexapodMjEnv.reset` with `randomize=True`) draws `set_servo_scale(hold=...)` from 0.8 to 1.2, so training sees ratios over the whole 2.0 to 3.0 estimate rather than the single placeholder.
Motor cap and friction are both multiples of stall, so `set_servo_scale(stall=0)` (limp legs) removes the holding torque too, and the `servo_margin.py` stall sweep weakens both.
The XML `ctrlrange` (`build_model.py` `CTRL_LIMIT = 1.5` N.m) sits above the largest randomized motor torque (about 0.82 N.m), so it never clips it; the `sim_sandbox.py` stall slider stops at 0.85 N.m, where the nominal motor share reaches 1.5 N.m.

## The measurement that replaces it

A bench test on one powered MG92B at 6 V, horn on a lever arm of known length:
1. Command a fixed angle, then hang increasing weights on the lever until the horn starts to turn backwards; that torque is the holding torque.
2. With the same lever, command a move that lifts the weight and find the largest weight it still lifts; that torque is the driving (stall) torque.
3. `SERVO_K_HOLD` is the ratio of the two, and `SERVO_STALL` is replaced by the measured driving torque if it differs from 0.34 N.m.
Repeat on a few servos and use the spread as the `hold` randomization range.

## Verification

`test_servo_model.py` checks the behaviour physically, in MuJoCo.
For loads of 1.05 x stall, the stall-hold midpoint and 0.95 x hold, a static joint holds (under 0.5 deg, under 0.02 deg of motion in the last second of 2 s), and a servo commanded against the load neither lifts it nor gives way by 0.5 deg in 2 s.
A load above hold back-drives the joint, the servo moves freely against a load below stall, stall=0 leaves no holding torque, and the `hold` scale lowers only the holding torque.
A 20-degree femur step in the standing robot, up and down, at nominal strength and at the four stall/hold corners of the randomization, arrives monotonically (never moving away from the target), overshoots by under 0.011 deg, ends within 0.05 deg and moves under 0.005 deg in its last 0.5 s of 8 s.
At 0.8 x stall the downward step, one leg lifting part of the body, takes about 4.1 s to arrive within 0.1 deg; at nominal strength about 2.2 s.

## Effect of the change (stall 0.50 N.m symmetric -> 0.34 N.m driving, 0.85 N.m holding)

Before this change the cap was a symmetric 0.50 N.m, 1.5x the rated stall and described as calibrated to observed behaviour, which looks like a stand-in for the missing holding torque.
The trained runs in `runs/` were trained and evaluated against the old servo model and have not been re-evaluated against this one.
`src/resources/speed_model.json` was regenerated with `measure_speed_model.py`; the slower gaits lose most of their lateral speed at full stick (WAVE vy 0.11 -> 0.02-0.03 m/s, now peaking at half stick with 0.08 m/s; RIPPLE rightward 0.14 -> 0.03 m/s), BI_GATE's top yaw rate drops from 2.13 to 1.82 rad/s, and TRI_GATE and TUNED change by under 10 %.

`servo_feasibility.py --runs` (open-loop gaits only; 3 terrains x 7 commands), motor torque as a fraction of the 0.34 N.m stall:

| arm | torque mean | torque p99 | above 95 % of stall | joint speed p99 | lag mean | lag p99 |
| --- | ----------- | ---------- | ------------------- | --------------- | -------- | ------- |
| firmware gait | 0.56 | 1.75 | 20.1 % | 0.33 | 1.40 deg | 14.90 deg |
| `__single___wide3` | 0.90 | 1.75 | 42.0 % | 0.60 | 8.75 deg | 55.45 deg |
| `__single__` | 1.05 | 1.75 | 53.1 % | 0.54 | 10.10 deg | 64.61 deg |
| `flat_tripod` | 1.03 | 1.75 | 51.4 % | 0.58 | 11.29 deg | 66.62 deg |
| `curb_140` | 1.00 | 1.75 | 49.7 % | 0.57 | 10.21 deg | 66.06 deg |

The p99 of 1.75 is the motor cap: the gaits use the motor's full share, while driving (the friction then takes 0.75 of stall back) as well as while holding or braking.
Mean tracking lag rose against the old model (firmware gait 0.75 -> 1.40 deg, `wide3` 6.22 -> 8.75 deg), and joint speed is still never near the no-load cap.

`servo_margin.py --runs` (3 terrains x 7 commands x 3 seeds), fraction of each arm's nominal score retained as stall (and with it hold) is scaled down:

| arm | x1.0 | x0.9 | x0.8 | x0.7 | x0.6 | x0.5 | falls |
| --- | ---- | ---- | ---- | ---- | ---- | ---- | ----- |
| firmware gait | 1.00 | 0.98 | 0.96 | 0.89 | 0.87 | 0.79 | 0 of 378 |
| `__single___wide3` | 1.00 | 0.98 | 0.97 | 0.98 | 0.96 | 0.93 | 0 of 378 |
| `__single__` | 1.00 | 0.96 | 0.96 | 0.92 | 0.89 | 0.89 | 0 of 378 |

Verdict: the tuned gaits keep at least 89 % of their score at half the (now lower) stall torque and nothing falls, so the earlier conclusion that `wide3` has the torque margin to port survives the asymmetric model.
The shipped firmware gait just crosses the 0.80 knee at x0.5 (0.79).
Nominal scores are firmware gait 0.522, `wide3` 0.864, `__single__` 0.759.
