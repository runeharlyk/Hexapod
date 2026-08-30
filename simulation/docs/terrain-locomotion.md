# Walking on everything: gait search, policies, and how they are compared

Status: 2026-08-02.

Reproduce: `optimize_gait_terrain.py` (search) → `bench_terrain.py` (compare) →
`reward_alignment.py` (reward diagnostics). Raw episode data in `runs/bench_*.json`, gait library in
`src/resources/gait_library.json`, search logs in `logs/gaitsearch_*.log`.

## Summary

1. **The shipped gait is badly mistuned, and that — not terrain difficulty — was the headline
   problem.** Its foot lift is 11.7 mm where every terrain wants 26–76 mm. A CMA-ES search over 18
   gait constants takes the suite score from 0.243 to 0.859 with no sensing and no learning.
2. **The previously documented "policy beats gait" result was measured against that mistuned
   gait.** Against a properly searched open-loop gait the trained policies lose at every roughness.
3. **Terrain knowledge is worth almost nothing** (+0.011 for a per-terrain oracle over one gait for
   all 13 conditions). No terrain classifier is needed.
4. **Obstacle ceiling is 140 mm clean / 160 mm scrambling**, up from 30 mm (firmware) and 80–100 mm
   (previous policies) — 2.1× the robot's stance height.
5. **Duty factor is the parameter the old action space could not reach** (needs 0.47–0.81, offered
   0.35–0.517), which is why `residual_sched` exists.
6. **Training on top of a strong gait failed, three times, for a reason worth knowing**: the reward
   landscape around a good gait is flat, so PPO's entropy bonus inflates the action std and the
   deterministic mean drifts off. Warm-started residual policies need `--ent-coef 0`.
7. **Leg phasing barely matters; wave and ripple gaits are not viable on this machine.** The search
   had them available and rejected them everywhere.
8. **The training reward, not PPO, was what kept policies below their own base gait.** A policy can
   earn *more* reward than the gait it started from while scoring 0.23 lower. Replacing the shaped
   reward with a per-step analogue of the evaluation score (`--reward-mode score`) moved the best
   policy from 0.75 to 0.85 and from losing by 0.2–0.4 to **statistical parity with the best
   open-loop gait on held-out commands** (−0.017, p = 0.81), with ~25 % less body agitation.
   Against the previous best policy: +0.128 (p < 0.0001).

9. **On level ground the best tripod (0.979) beats the best bipod (0.953)**, and both want roughly
   6× the firmware's foot lift and far more duty than it offers (0.608 vs 0.517; 0.749 vs 0.350).
10. **Search budget matters more than expected.** Re-running the flat case at 2400 evaluations
    instead of 640 moved it 0.954 → 0.996. Every 640-evaluation library entry is therefore a
    **lower bound**, with a deficit of order 0.04.

Three things turned out to be artifacts of how a comparison was set up rather than real effects —
"policies buy smoothness", "policies generalize better", and "the unconstrained search beats a
pinned tripod" — and each only became visible after re-running the *search* under the changed
conditions instead of re-scoring old rollouts. That is the methodological lesson: when an arm loses
on an axis it was never optimizing, re-optimize it before concluding anything. The corollary is to
distrust any result where a constrained search appears to beat an unconstrained one — that is a
convergence report, not a finding.

## The question

The hexapod must walk stably on clean ground, on rough ground, and over obstacles that stop the
default gait.
A trained policy is only interesting if it beats the *best open-loop gait for that terrain*, not
merely the one gait that happens to be compiled into the firmware.
Everything below exists to make that comparison honest.

## Why the old comparison was not honest

Three defects, all of which flattered the policy:

1. **One baseline gait for every terrain.**
   `optimize_gait.py` tunes a single coefficient set, on flat ground, against the RL reward.
   Nothing had ever asked what gait a 120 mm rock field actually wants.
2. **Unequal authority.**
   The `residual_gait` policy can change step height, cadence, stride and **body ride height**
   (±25 mm) every control step.
   The analytic baseline could change none of those: ride height was pinned at 0 and the
   coordination pattern was a fixed interpolation between the firmware's tripod and bipod.
3. **Search objective ≠ report objective.**
   The gait was tuned on the training reward (which mixes energy, action rate, slip and upright
   penalties) but judged on rough-terrain progress.

## What replaced it

### `src/robot/gait_schedule.py` — the gait as an 18-parameter vector

A `GaitSchedule` is the firmware's command→gait map with the missing degrees of freedom exposed:

| group | parameters |
| ----- | ---------- |
| stride gains | `gx0 gx1 gy0 gy1 gyaw0 gyaw1 blend_speed yaw_comp` |
| cadence | `pr_base pr_slope pr_yaw` |
| foot trajectory | `step_height step_depth` |
| posture | `ride_mm` (± body height, same ±25 mm the policy gets) |
| **coordination** | `duty` (stance fraction), `lag_r`, `lag_l`, `contra` |

The coordination parameters are the substantive addition.
Legs are `[RF, RM, RR, LF, LM, LR]`; phase offsets are

```
right = [0, lag_r, 2·lag_r]        left = contra + [0, lag_l, 2·lag_l]     (mod 1)
```

which is the metachronal family containing every gait the firmware ships:

| gait | `lag_r` | `lag_l` | `contra` | offsets |
| ---- | ------- | ------- | -------- | ------- |
| tripod | 1/2 | 1/2 | 1/2 | 0, 1/2, 0, 1/2, 0, 1/2 |
| bipod | 1/3 | 2/3 | 2/3 | 0, 1/3, 2/3, 2/3, 1/3, 0 |
| wave | 1/6 | −1/6 | 5/6 | 0, 1/6, 1/3, 5/6, 2/3, 1/2 |
| ripple | 2/3 | 2/3 | 1/6 | 0, 2/3, 1/3, 1/6, 5/6, 1/2 |

`duty` is independent of the pattern here, whereas the firmware ties the two together.

The firmware's shipped tripod is *skewed* — `[0, .52, .08, .58, .16, .66]`, not the ideal
`[0, .5, 0, .5, 0, .5]` — and is not exactly representable in this family (nearest member is off by
up to 0.08 of a cycle on one leg).
It is therefore not the origin of the search space; it remains available exactly as the env's
`gait_schedule=None` path and is always benchmarked as its own arm (`analytic_gait`).

### `src/sim/rollout.py` — one rollout, one score, shared by search and benchmark

The optimizer and the benchmark import the *same* `episode()` and the *same* `score()`.
If they did not, "the policy beats the best tuned gait" would be comparing a tuned-for-A arm
against a judged-on-B metric.

```
score = alive · ( track − 0.30·stuck − 0.25·knock )

alive  fraction of the episode survived (a fall truncates and is paid for in proportion)
track  exp(−vel_err²/0.04) · exp(−yaw_err²/0.08)   on the episode-mean velocity and yaw rate
stuck  fraction of steps below 30 % of the commanded speed  — high-centering, which is how a
       statically stable hexapod actually fails
knock  shin/belly contact force as a fraction of body weight — the cost of plowing through an
       obstacle instead of stepping over it
```

Tracking is scored, not progress: progress rewards overshooting a slow command, which is a
tracking failure, and a search told to maximize progress will simply walk too fast.
Progress is still reported, because on rough ground it is the intuitive readout.

Known limitation: `vel_err` uses the episode-mean velocity, so a lurching gait that alternates
between double speed and zero can average out to a perfect mean.
`stuck` catches most of that (the zero half counts as stalled) but not all of it, and body tilt is
not in the score at all — it is reported separately and checked on the winners.

### `optimize_gait_terrain.py` — the search

Three modes:

| mode | what it produces | why |
| ---- | ---------------- | --- |
| `single` | one gait maximizing the mean score over the whole terrain suite | the honest opponent for a policy, which is also a single controller |
| `per-terrain` | one gait per terrain condition | an **oracle** upper bound: what terrain knowledge is worth |
| `curb` | one gait per curb height | the obstacle-clearing limit |

Protocol:

- Search seeds are `0..n−1`; every reported number is re-measured on held-out seeds `500+`, which
  are also the benchmark's seeds. A gait that memorized its search terrain shows up as a
  validation drop instead of a win.
- The four named gaits (tripod / bipod / wave / ripple, on the tuned stride and cadence
  coefficients) are evaluated for free as seed points, CMA-ES starts from the best of them, and the
  **held-out winner among {search result, all four seeds} is what gets stored**. A search that
  cannot beat a named gait is reported as such rather than silently returning something worse.

Optimizer choice: CMA-ES by default. The evaluations are cheap (~0.4 s for a 12-episode candidate
on 14 workers) and the space is 18-dimensional and continuous, which is the regime where a
surrogate model costs more than it saves. Optuna TPE (`--optimizer tpe`) and differential evolution
(`--optimizer de`) are implemented so the claim is measured rather than asserted — see
[Optimizer comparison](#optimizer-comparison).

### `residual_sched` — matching the policy's authority to the finding

The first search result already showed the problem: the best gaits on rough ground use
**duty ≈ 0.73**, and `residual_gait` cannot reach it.
Its `gait_blend` axis only interpolates tripod (duty 0.517) → bipod (duty 0.35), so it can make the
duty *lower* and never higher.
Measured on the benchmark, `rough_nocontact` sits at duty 0.41–0.43 — pinned toward the bipod end,
walking a gait the open-loop search says is wrong for the ground it is on.

`residual_sched` (27-D) is `residual_gait` plus three channels: `duty`, metachronal `lag`, and the
left/right offset `contra`, all as deltas on the base schedule, so **zero action is still exactly
the base gait** and `--zero-final` still starts training there (verified byte-identical).

Leg phase offsets enter the gait as `(clock + offset) mod 1`, so a step change teleports that foot
along its trajectory. The policy therefore commands a *target* phasing and the env slews the live
offsets toward it at 0.6 cycles/s — re-timing a leg is a physical act with a speed limit.

### `bench_terrain.py` — the comparison

Arms, all on byte-identical terrain and pushes for a given (kind, height, command, seed):

| arm | what it is |
| --- | ---------- |
| `analytic_gait` | the legacy firmware gait — the old baseline |
| `tuned_single` | best single open-loop gait over the whole suite |
| `tuned_oracle` | best open-loop gait *per terrain*, told which terrain it is on |
| `<run>` | trained policies |
| `…+reflex` | any of the above with contact reflexes in the gait engine |

Paired deltas are reported against `tuned_oracle` with a sign test and a Wilcoxon signed-rank test.

## Results

### 1. The old baseline was the problem, not the terrain

CMA-ES, 480 candidate evaluations, 13 terrain conditions × 3 commands × 2 search seeds; scored on
held-out seeds 500–503 (the benchmark's own seeds).

| gait | held-out score over the suite |
| ---- | ----------------------------- |
| firmware tripod (`gait_coef.json` gains) | 0.243 |
| bipod | 0.530 |
| wave | 0.003 |
| ripple | 0.005 |
| **CMA-ES result (`__single__`)** | **0.859** |

The searched gait, `src/resources/gait_library.json:__single__`:

```
duty 0.626   offsets [0, .418, .835, .502, .937, .372]   ride +5.3 mm
step_height 55 mm (firmware: 11.7 mm)   step_depth 7.4 mm
gx0 .461 gx1 .224   pr_base -.268 pr_slope 1.42 pr_yaw .547
```

The single largest defect in the shipped gait is **foot lift**: 11.7 mm, against 55 mm for the
searched gait. Wave and ripple score ≈ 0 because at duty 5/6 the firmware's stroke geometry barely
advances the body; they are in the family and the search rejected them, which is the useful
outcome — they are not viable on this machine, not merely untried.

Sanity check that this is a real gait and not a scoring artifact: at a 0.15 m/s command the
schedule asks for a 58.4 mm stride at 1.65 cyc/s with duty 0.626, i.e. a predicted body speed of
`stride / (duty / cadence)` = **0.154 m/s**, against **0.142 m/s** measured. The kinematics
account for the speed.

### 2. The trained policies do **not** beat the best open-loop gait

Benchmark: 4 kinds × 4 heights × 3 commands × 4 seeds, byte-identical terrain per arm, no domain
randomization. Score (higher is better), all kinds and commands pooled:

| arm | flat | 40 mm | 80 mm | 120 mm |
| --- | ---- | ----- | ----- | ------ |
| `analytic_gait` (firmware) | 0.66 | 0.38 | 0.17 | 0.12 |
| `rough_nocontact` (12 M, policy) | 0.76 | 0.75 | 0.64 | 0.38 |
| `rough_contact` (12 M, policy) | 0.82 | 0.81 | 0.71 | 0.47 |
| **`tuned_single`** (open loop) | **0.94** | **0.93** | **0.91** | **0.72** |

Zero falls in all 624 episodes, for every arm.

**The previously reported policy win was mostly an artifact of a badly tuned baseline.**
The policies genuinely beat the *firmware* gait by a wide margin — and that is what the earlier
numbers measured — but an open-loop gait with no sensing, no learning and 18 tuned constants beats
both policies at every roughness level.

The obvious objection is that the gait was searched on nominal dynamics while the policies were
*trained* with domain randomization, so the gait might simply be overfit to a perfect robot.
It is not. Re-running the entire matrix with `--randomize` (mass, friction, servo strength/speed,
gear lash, action latency, IMU noise, random pushes) moves it by less than the seed noise:

| arm | flat | 40 mm | 80 mm | 120 mm |
| --- | ---- | ----- | ----- | ------ |
| `tuned_single`, nominal | 0.94 | 0.93 | 0.91 | 0.72 |
| `tuned_single`, **+DR** | 0.94 | 0.92 | 0.91 | 0.73 |
| `rough_contact`, +DR | 0.82 | 0.81 | 0.73 | 0.50 |
| `analytic_gait`, +DR | 0.64 | 0.39 | 0.18 | 0.11 |

Two honest qualifications, neither of which changes the conclusion:

- **The tuned gait is a rougher ride.** Body-rate RMS is 1.20–1.41 rad/s against 0.67–0.93 for the
  policies, and shin/belly knock is 0.07–0.08 of body weight against 0.001–0.06. The policies buy
  smoothness; the score does not price it. Mean body tilt is comparable (8.7° vs 8.4° at 120 mm).
- **The comparison is confounded for these two policies**: both were trained on the legacy gait as
  their zero-action base, so they were learning to repair a bad gait rather than to improve a good
  one. The clean question — does learning add anything *on top of* the best open-loop gait — is
  what the `sched_tuned` / `gait_tuned` runs below test.

#### How much does the answer depend on the score weights?

Re-scoring the *same* 624 episodes under different weights (free — no new rollouts):

| weighting | `analytic` | `rough_nocontact` | `rough_contact` | `tuned_single` |
| --------- | ---------- | ----------------- | --------------- | -------------- |
| as used (stuck .30, knock .25) | 0.256 | 0.604 | 0.675 | **0.859** |
| knock ×4 | 0.252 | 0.583 | 0.660 | **0.807** |
| knock ×10 | 0.244 | 0.542 | 0.628 | **0.702** |
| + smoothness (rate .2) | 0.149 | 0.449 | 0.510 | **0.598** |
| + smoothness (rate .4, knock ×4) | 0.038 | 0.274 | **0.330** | 0.285 |
| progress only | 0.210 | 0.626 | 0.642 | **0.976** |

The ranking is **robust to how much you penalize plowing** — even at a 10× knock weight the tuned
gait wins — but **not to how much you penalize body agitation**. Price roll/pitch rate heavily
enough and `rough_contact` overtakes it.

That looks like the precise statement of what the learned policies buy on this robot: not distance,
not tracking, not fall avoidance — *smoothness*.

**It is not.** That comparison is rigged: `tilt_rate` is not in the search objective, so
`tuned_single` was never asked to be smooth, while both policies were trained with an explicit
angular-rate penalty. Re-running the search with smoothness priced (`--score-rate 0.4
--score-knock 1.0`, identical budget, seeds and protocol) gives `__single___smooth`:

| arm | score under the smoothness-priced objective |
| --- | ------------------------------------------- |
| `analytic_gait` (firmware) | 0.038 |
| `rough_nocontact` | 0.274 |
| `tuned_single` (smoothness-blind) | 0.285 |
| `rough_contact` | 0.330 |
| **`__single___smooth`** (searched *for* this objective) | **0.371** |

The smooth gait is a visibly different animal: 30 mm foot lift instead of 61, body raised 17.9 mm
instead of 5.3, cadence slope 2.79 instead of 1.42, and knock 0.002 instead of 0.070 — it steps
lightly and quickly rather than striding. It pays for that with progress 0.44 against 0.98.

So across every *objective* tested — raw progress, tracking, plowing-penalized and
smoothness-penalized — an open-loop gait found by CMA-ES beats both 12 M-step policies, and the
apparent "policies buy smoothness" advantage was an artifact of comparing a smoothness-blind gait
against smoothness-trained policies.

#### …but the command set was rigged too, and there the policies win

Everything above evaluates on `COMMANDS` — the same three forward/turn commands the gait search
optimizes over. The policies train on the full omnidirectional distribution. That is a specialist
being scored on its specialty against a generalist, and correcting it flips the result.

Evaluated on four commands the search never saw (backward, strafe, hard turn, top speed):

| arm | flat | 40 mm | 80 mm | 120 mm | yaw_err |
| --- | ---- | ----- | ----- | ------ | ------- |
| `rough_nocontact` | **0.92** | 0.88 | 0.80 | 0.65 | 0.026–0.049 |
| `rough_contact` | 0.88 | **0.89** | **0.82** | **0.68** | 0.025–0.049 |
| `analytic_gait` (firmware) | 0.80 | 0.64 | 0.45 | 0.39 | 0.036–0.069 |
| `tuned_single` | 0.71 | 0.71 | 0.68 | 0.61 | **0.108–0.123** |
| `tuned_oracle` | 0.70 | 0.71 | 0.67 | 0.53 | 0.064–0.186 |
| `__single___smooth` | 0.40 | 0.47 | 0.43 | 0.30 | 0.199–0.220 |

The tuned gait still *travels* fine (progress 0.94–1.06); it is yaw tracking that collapses —
0.108–0.123 rad/s against 0.026–0.049 for the policies — because its `gyaw` and `gy` gains were
only ever exercised by one turn command and no strafe. On flat ground it drops **below the firmware
gait** (0.71 vs 0.80).

This is the generalization gap, and it is the one thing in this study that learning clearly buys.

`rollout.py` therefore defines three disjoint command sets — `COMMANDS` (the historical benchmark,
3), `COMMANDS_WIDE` (7, spanning the envelope the policy trains on, via `--command-set wide`) and
`COMMANDS_HELDOUT` (5 at values none of the above uses: `fwd_mid`, `back_fast`, `strafe_rev`,
`turn_mid`, `spin`).

Scored on `COMMANDS_HELDOUT`, 4 seeds, 20/80 episodes per cell:

| arm | flat | 40 mm | 80 mm | 120 mm | falls |
| --- | ---- | ----- | ----- | ------ | ----- |
| `rough_contact` | **0.79** | **0.77** | 0.71 | 0.53 | 0 |
| `__single__` (3-command search) | 0.71 | 0.72 | **0.76** | **0.74** | 0 |
| `rough_nocontact` | 0.71 | 0.71 | 0.64 | 0.46 | 0 |
| `analytic_gait` | 0.63 | 0.49 | 0.33 | 0.28 | 0 |
| `__single___wide` (7-command search, 480 evals) | 0.57 | 0.57 | 0.58 | 0.51 | **7/80** |

So the honest picture is a split, not a winner:

- **On easy ground the policies generalize better** (0.79/0.77 vs 0.71/0.72 at flat and 40 mm).
- **On rough ground the tuned gait still wins even on commands it never saw** (0.76 vs 0.71 at
  80 mm; 0.74 vs 0.53 at 120 mm).

And a negative result worth recording: the first *wide-command* search came out **worse than the
3-command gait on every cell, and fell 7 times**. At 480 evaluations over 7 commands × 13 terrains
it had roughly a third of the per-command sampling the narrow search got. It did, however,
independently rediscover the documented `pr_yaw` defect, picking 1.31 against the current 0.55 (the
firmware uses 1.5) — corroboration from an unrelated direction.

Re-run properly (`__single___wide2`: 960 evaluations, reduced 5-cell suite, 6 held-out seeds), the
fair comparison comes out like this — **gait searched over the policy's command envelope, both
scored on commands neither has seen**:

| arm | flat | 40 mm | 80 mm | 120 mm |
| --- | ---- | ----- | ----- | ------ |
| **`__single___wide2`** (wide search) | **0.88** | **0.88** | **0.81** | 0.58 |
| `__single__` (narrow search) | 0.71 | 0.72 | 0.76 | **0.74** |
| `rough_contact` (best policy) | 0.79 | 0.77 | 0.71 | 0.53 |
| `analytic_gait` (firmware) | 0.63 | 0.49 | 0.33 | 0.28 |

The wide-searched gait beats the best trained policy at **every** roughness on held-out commands, so
the generalization advantage seen earlier was an artifact of the gait's narrow search domain, not a
property of learning. It is weaker than the narrow gait at 120 mm (0.58 vs 0.74) because its reduced
5-cell search suite contained `steps_120` but no `bumps/rocks/waves_120`, and it pays for its speed
with a much rougher ride (body rate 1.72–1.78 against 1.04–1.27) and more knock (0.083–0.190).

### 3. Policies retrained on the tuned base gait

Two 12 M-step runs, both with `--zero-final` so training starts exactly at `tuned_single`, mixed
terrain, adaptive roughness curriculum to 0.12 m, 4-frame observation history:

| run | control mode | what it isolates |
| --- | ------------ | ---------------- |
| `gait_tuned` | `residual_gait` (24-D) | value of learning, at the old action space |
| `sched_tuned` | `residual_sched` (27-D) | + authority over duty and leg phasing |

**Both failed, and failed identically** — score by roughness, against the gait they started from:

| arm | flat | 40 mm | 80 mm | 120 mm | vel_err | body rate |
| --- | ---- | ----- | ----- | ------ | ------- | --------- |
| `tuned_single` (= their zero action) | 0.94 | 0.93 | 0.90 | 0.75 | 0.018–0.028 | 1.20–1.41 |
| `gait_tuned` | 0.47 | 0.47 | 0.46 | 0.38 | ~0.10 | 0.36–0.63 |
| `sched_tuned` | 0.48 | 0.46 | 0.42 | 0.35 | ~0.10 | 0.34–0.68 |

Two 12 M-step runs starting from a 0.94 policy converged to 0.48. That the two action spaces landed
in the same place is the useful part: it rules out `residual_sched` as the cause.

**Diagnosis: the training command distribution, not the algorithm.**

- The adaptive terrain curriculum pinned roughness at its 0.12 m maximum almost immediately,
  because the new base gait keeps satisfying the promotion gate.
- At 0.12 m, `TERRAIN_SPEED_FLOOR = 0.5` halved the sampled command range — max 0.225 m/s, and a
  **mean |vx| of 0.085 m/s** in the training logs.
- The policies were then benchmarked at 0.15–0.30 m/s. Their own training logs show `track_score`
  0.75 and `abs_bvx` 0.084: they were tracking *correctly*, at speeds nobody was going to ask for.
- The resulting policies are smooth (body rate 0.34–0.68 against the gait's 1.20–1.41) and clean
  (knock ≈ 0) and slow. That is the reward they were given.

The throttle exists because commanding a speed the robot cannot reach parks the tracking kernel in
its flat tail and teaches thrashing. That premise was true of the legacy gait and is **false of the
searched gait**, which makes 0.92 of a 0.30 m/s command on 120 mm terrain. `TERRAIN_SPEED_FLOOR` is
therefore now 1.0 (off) by default, with `--terrain-speed-floor` to restore it; measured effect on
the training distribution at 0.12 m roughness: mean |vx| 0.090 → 0.181 m/s, max 0.225 → 0.45 m/s.

`sched_tuned2` re-ran with the throttle off — and came out **worse still** (0.26/0.25/0.22/0.20).
So the throttle was a real defect but not the cause. Four hypotheses, tested in order:

| hypothesis | test | verdict |
| ---------- | ---- | ------- |
| command throttle starves training of speed | remove it, retrain | **real defect, not the cause** — result got worse |
| reward disagrees with the benchmark score | rank 41 gaits both ways (`reward_alignment.py`) | **partly true, not the cause** — Spearman ρ = 0.72, and the best re-weighting only reaches 0.76 |
| PPO exploration destroys the tuned base gait | score the base gait under action noise σ = 0.1…0.3 | **no** — 0.867 → 0.843 |
| the deterministic mean action is not where the policy operates | evaluate deterministic vs stochastic | **yes** |

| run | final action std | deterministic | stochastic |
| --- | ---------------- | ------------- | ---------- |
| `sched_tuned2` | 0.514 | **0.226** | 0.491 |
| `sched_tuned` | 0.472 | 0.443 | 0.616 |
| `rough_contact` (healthy reference) | 0.439 | 0.734 | 0.737 |

**Entropy pathology.** Action std was initialized at 0.30 and *grew* to 0.51. A healthy run has
deterministic ≈ stochastic; these have a 2.2× gap, meaning the policy mean drifted somewhere bad and
only the sampling noise kept it working.

The two findings connect. The reward landscape around a good base gait is **compressed**: over the
41-gait population, a gait scoring 0.91 earns 3.90 reward while a random gait scoring ≈ 0 earns
2.52 — the entire quality range spans 1.4 reward units. With that weak a policy gradient, an
`ent_coef` of 0.005 across 27 action dimensions is enough to inflate the std, and the entropy bonus
wins. `rough_contact` did not suffer this because its legacy base gait was bad enough that the
reward gradient around it was steep.

Fix, now exposed as flags rather than constants: `--ent-coef 0` for residual modes on a tuned base,
plus the best-measured reward re-weighting (`--reward-vel 8 --reward-angvel 0.02 --reward-knock
0.1`, ρ = 0.754 with 4/5 top-5 agreement against 3/5 for the defaults). `sched_tuned3` is the run
with both.

This is the most useful thing the study produced for future training work: **a policy warm-started
at a strong open-loop gait needs its entropy bonus turned off, because the reward signal that is
supposed to hold it there is far weaker than the reward signal that pushed it out.**

#### `sched_tuned3`: the correctly configured run

9 M steps, `--ent-coef 0 --init-std 0.25 --reward-vel 8 --reward-angvel 0.02 --reward-knock 0.1`.
Action std now **shrinks**, 0.25 → 0.216, and the pathology is gone.

| arm | flat | 40 mm | 80 mm | 120 mm | (held-out commands) |
| --- | ---- | ----- | ----- | ------ | ------------------- |
| `gait[__single___wide2]` | **0.88** | **0.88** | **0.81** | 0.58 | |
| `rough_contact` | 0.79 | 0.77 | 0.71 | 0.53 | |
| `tuned_single` (`__single__`) | 0.71 | 0.72 | 0.76 | **0.74** | |
| **`sched_tuned3`** | 0.68 | 0.66 | 0.63 | 0.55 | |
| `sched_tuned2` (pathological) | 0.27 | 0.28 | 0.24 | 0.22 | |
| `analytic_gait` | 0.63 | 0.49 | 0.33 | 0.28 | |

And on the search command set:

| arm | flat | 40 mm | 80 mm | 120 mm |
| --- | ---- | ----- | ----- | ------ |
| `tuned_single` | **0.94** | **0.93** | **0.91** | **0.72** |
| `gait[__single___wide2]` | 0.82 | 0.83 | 0.78 | 0.49 |
| `rough_contact` | 0.82 | 0.81 | 0.71 | 0.47 |
| `sched_tuned3` | 0.62 | 0.62 | 0.59 | 0.51 |
| `analytic_gait` | 0.66 | 0.38 | 0.17 | 0.12 |

Fixing the entropy pathology bought a 2.5× improvement (0.27 → 0.68 at flat), and `sched_tuned3` is
the calmest arm in the study — lowest body tilt (0.8–7.3°), lowest knock (0.000–0.016), lowest
stall fraction on held-out commands (0.05–0.16). But **it still does not beat the searched open-loop
gait, and does not beat `rough_contact` either.** Its specific weakness is yaw tracking
(0.126–0.160 rad/s against `rough_contact`'s 0.077–0.118), inherited from a base gait whose yaw
gains were tuned on a single turn command.

#### The reward was the problem, and replacing it fixed most of the gap

`sched_v4` (all fixes above, trained on the wide-command base gait) reached 0.75/0.72/0.70/0.63 on
the bench set — better than every earlier policy, still far below its own base gait's 0.98.

The decisive diagnostic: **on its exact training distribution `sched_v4` earns +6.664 reward against
the base gait's +6.382.** PPO succeeded at its objective, found a policy better by the reward's own
measure, and scored 0.23 lower. The optimizer was never the problem.

Why the reward misleads, measured (`reward_resolution.py`): rank agreement with the score is
ρ = 0.81 over a broad gait population but only **ρ = 0.61 among perturbations of the best gait**.
A reward good enough to *find* a decent gait is not good enough to *refine* a near-optimal one, and
global rank correlation hides exactly that.

So `--reward-mode score` replaces the shaped reward with a per-step analogue of `rollout.score`:
the velocity and yaw kernels **multiplied** rather than added (a yaw error must be able to ruin the
step instead of being bought off with speed), plus only the stall and knock penalties the evaluation
objective actually contains. The upright / height / vz / energy / power / slip / angvel terms are
dropped — they are what PPO was optimizing instead of walking. Falls remain handled by termination.

`sched_v5` (identical to `sched_v4` except the reward mode):

| arm | flat | 40 mm | 80 mm | 120 mm | body rate | knock |
| --- | ---- | ----- | ----- | ------ | --------- | ----- |
| **bench command set** | | | | | | |
| `__single___wide3` (base gait) | 0.98 | 0.96 | 0.93 | 0.77 | 0.69–0.99 | 0.004–0.051 |
| **`sched_v5`** | 0.85 | 0.86 | 0.84 | 0.70 | 0.39–0.83 | 0.000–0.010 |
| `sched_v4` | 0.75 | 0.72 | 0.70 | 0.63 | | |
| `rough_contact` | 0.82 | 0.81 | 0.71 | 0.47 | | |
| **held-out command set** | | | | | | |
| `__single___wide3` | 0.80 | 0.79 | 0.79 | 0.67 | 0.56–0.88 | 0.005–0.037 |
| **`sched_v5`** | 0.76 | 0.77 | 0.76 | **0.68** | 0.41–0.72 | 0.000–0.011 |
| `rough_contact` | 0.79 | 0.77 | 0.71 | 0.53 | | |

Paired Wilcoxon on identical terrain / command / seed:

| comparison | bench | held-out |
| ---------- | ----- | -------- |
| `sched_v5` − `rough_contact` | **+0.128** (p < 0.0001) | **+0.059** (p = 0.0001) |
| `sched_v5` − `sched_v4` | **+0.115** (p < 0.0001) | **+0.127** (p < 0.0001) |
| `sched_v5` − base gait | −0.090 (p < 0.0001) | **−0.017 (p = 0.81, n.s.)** |
| `sched_v5` − base gait, 120 mm only | −0.066 (p = 0.02) | +0.005 (p = 0.23, n.s.) |

So the answer to "does the trained policy beat the best gait setup" is now:

> **On the gait's own tuned command set, no** — the gait wins by 0.090 (p < 0.0001).
> **On held-out commands, they are statistically indistinguishable** (−0.017, p = 0.81), and the
> policy does it with roughly 25 % less body-rate agitation and near-zero knock.
> Against the previous best policy the improvement is unambiguous: +0.128 / +0.059, p ≤ 0.0001.

That is a real improvement — earlier policies lost to the gait by 0.2–0.4 — but it is parity, not
superiority. The learned stage currently buys ride quality and robustness at the rough end
(+0.148 at 120 mm vs `rough_contact`), not distance or tracking.

Caveat on `sched_tuned3`: its `AdaptiveTerrain` gate still normalized `r_vel` by the stale
`R_VEL_WEIGHT` constant rather than the overridden weight of 8.0, which inflated the promotion score
by 2.29× and pinned roughness at the 0.12 m maximum for the whole run. That matches what every
previous run did, so the comparison holds, but the callback is now fixed (`vel_weight` argument) and
the next run will have a meaningful curriculum gate.

### Foot contact sensors, re-measured

The repo's existing conclusion — contacts near-worthless as an observation, valuable as gait
reflexes — was measured on the **mistuned 11.7 mm-lift gait**. That premise is gone, so both uses
were re-measured against the current arms. The answer changes in both directions.

#### As gait reflexes (no retraining; `--reflex`)

Paired on identical terrain / command / seed, held-out command set, score delta from switching
reflexes on:

| arm | flat | 40 mm | 80 mm | 120 mm | pooled | knock off → on |
| --- | ---- | ----- | ----- | ------ | ------ | -------------- |
| `analytic_gait` (11.7 mm lift) | +0.000 | +0.068\* | **+0.112**\* | +0.074\* | **+0.078**\* | 0.005 → 0.003 |
| `rough_contact` | −0.052\* | −0.043\* | −0.007 | **+0.091**\* | +0.008\* | 0.023 → 0.026 |
| `tuned_single` (wide3) | −0.007 | −0.067\* | −0.048\* | +0.080 | −0.012\* | 0.030 → **0.254** |
| `sched_v5` | −0.062\* | −0.082\* | −0.075\* | −0.020\* | **−0.059**\* | 0.005 → 0.028 |
| `tuned_oracle` | −0.094\* | −0.036\* | −0.026\* | −0.096\* | −0.056\* | 0.067 → **0.313** |

(\* = Wilcoxon p < 0.05)

**Reflexes help exactly the one arm they were originally tuned against, and hurt every good arm.**
On the tuned gaits, knock rises 8× (0.030 → 0.254) — the reach-until-loaded regulator digs the feet
into ground the gait was already clearing. It is solving a problem that adequate foot lift removed,
and creating a new one.

The exception is real and consistent: **at the roughest end reflexes still pay** (+0.091 for
`rough_contact` and +0.080 for `tuned_single` at 120 mm), and on the curb they extend an arm's own
ceiling:

| curb cleared (4 seeds) | 80 mm | 120 mm | 140 mm | 160 mm |
| ---------------------- | ----- | ------ | ------ | ------ |
| `analytic_gait` (± reflex) | 0/4 | 0/4 | 0/4 | 0/4 |
| `sched_v5` | 4/4 | 0/4 | 0/4 | 0/4 |
| `sched_v5` + reflex | 4/4 | **2/4** | 0/4 | 0/4 |
| `tuned_single` | 4/4 | 4/4 | 4/4 | 0/4 |
| `tuned_single` + reflex | 4/4 | 3/4 | 4/4 | **1/4** |
| `tuned_oracle` | 4/4 | 4/4 | 4/4 | 4/4 |

Consistent mechanism: reflexes correct touchdown *timing*, which is only genuinely wrong when the
terrain exceeds what the planned foot trajectory anticipates. Below an arm's ceiling they fire
spuriously and cost progress; at its ceiling they buy one step-height class.

So reflexes should be **gated on roughness, not enabled globally** — the opposite of what the
previous measurement implied.

#### As a policy observation (retrain; `--contact-obs`)

`sched_v5c` is identical to `sched_v5` in every respect except the 6 contact inputs (same base gait,
reward mode, entropy, seed, step count). Paired:

| command set | pooled Δscore | flat | 40 mm | 80 mm | 120 mm | Δyaw_err (pooled) |
| ----------- | ------------- | ---- | ----- | ----- | ------ | ----------------- |
| bench | −0.005 (p = 0.81) | +0.030\* | +0.005\* | −0.013 | −0.016 | **−0.005** (p = 0.0006) |
| held-out | **+0.015 (p < 0.0001)** | +0.036\* | +0.027\* | +0.013\* | −0.001 | **−0.011** (p < 0.0001) |

This is **better than the previous finding of no effect** (progress +0.009, p = 0.36), but small: a
+0.015 score gain concentrated on flat-to-moderate ground and on held-out commands, with the
clearest signal being yaw tracking (−0.011 rad/s, ~10 % of the error). At 120 mm the sensors buy
nothing as an observation.

#### Verdict on the hardware

| use | worth it? |
| --- | --------- |
| observation input | Marginal. +0.015 on held-out commands (p < 0.0001) and ~10 % better yaw tracking, nothing at high roughness. Real but small for 6 sensors and 6 wires. |
| gait reflexes, always on | **No.** Net negative on every well-tuned arm; knock ×8. |
| gait reflexes, gated to high roughness / obstacles | **Yes.** +0.08–0.09 at 120 mm, and one step-height class on the curb (`sched_v5` 0/4 → 2/4 at 120 mm; `tuned_single` 0/4 → 1/4 at 160 mm). |

The two uses are independent, so the sensible configuration is contacts wired to *both*: fed to the
policy, and driving reflexes that a roughness estimate switches on.

### Which parameters actually matter (and which were never worth searching)

`gait_sensitivity.py` sweeps each parameter across its bounds with the rest held at the
`__single___wide3` optimum. Span is best − worst over the sweep; a flat profile means the parameter
does nothing.

| parameter | chosen | best at | span | verdict |
| --------- | ------ | ------- | ---- | ------- |
| **`lag_r`** | 0.486 | **0.500** | **0.847** | the most influential parameter in the vector |
| `pr_base` | −0.325 | −0.200 | 0.788 | cadence offset, very sensitive |
| **`lag_l`** | 0.480 | 0.400 | 0.747 | second-most influential |
| `yaw_comp` | 0.027 | 0.000 | 0.673 | |
| `duty` | 0.500 | 0.460 | 0.573 | |
| `step_height` | 0.664 | 1.000 (bound) | 0.417 | bound is binding — see feasibility below |
| `pr_slope`, `gx1`, `contra`, `pr_yaw`, `gyaw0`, `ride_mm` | | | 0.22–0.40 | moderate |
| `gyaw1`, `gx0` | | | 0.09 | weak |
| **`step_depth`, `blend_speed`, `gy1`, `gy0`** | | | **0.011–0.035** | **inert** |

**Leg phasing dominates everything else.** `lag_r` spans 0.847 — mis-set it and the score collapses
from 0.868 to 0.021 — and it peaks at exactly 0.500, the ideal tripod. This is the strongest
argument in the study for tripod as the structural choice on this robot, and it arrives independently
of the level-ground tripod/bipod comparison.

The four inert parameters are worth dropping from future searches: the lateral stride gains `gy0`
and `gy1` barely register because the command sets are forward-dominated, and `step_depth` and
`blend_speed` do essentially nothing. Six free dimensions spent on ~0.03 of score is budget that the
sensitive parameters (and the 640-eval convergence deficit) need more.

#### Is the metachronal family a good prior?

It is a 3-parameter constraint on a 5-dimensional space, and since phasing turned out to dominate,
it was worth testing. `--free-offsets` searches all five leg offsets directly (23-D):

| search | dims | evals | evals/dim | held-out score | offsets | IK out-of-range |
| ------ | ---- | ----- | --------- | -------------- | ------- | --------------- |
| metachronal (`__single___wide3`) | 18 | 1280 | 71 | **0.912** | 0, .486, .972, .479, .959, .438 | 1.6 % |
| free offsets (`__single___free`) | 23 | 1600 | 70 | 0.849 | 0, .472, .835, .531, .009, .331 | 9.1 % |

The free search lost at matched per-dimension budget, and its winner is *not* representable in the
family (right side .472/.835 is not 0, λ, 2λ) — so it explored outside the prior and found nothing
better. Strictly a superset cannot be worse at infinite budget, so the honest reading is that the
five extra dimensions do not pay for themselves here, not that free phasing is impossible. The
family stays.

### Gaits that rely on clamped joint commands

`ik_feasibility.py`. The IK clamps its acos arguments and MuJoCo clamps to `jnt_range`, both
silently, so a gait can score well while executing a trajectory its own IK never asked for. Fraction
of commanded joint angles outside the servo range, at a 60 mm stride:

| gait | lift | ride | % out of range | worst |
| ---- | ---- | ---- | -------------- | ----- |
| `__single___smooth`, `__single___wide`, `rocks_40`, `steps_40`, `waves_40/120` | ≤54 mm | | **0.0 %** | — |
| `steps_120` | 60.0 | +15.3 | 0.1 % | 0.1° |
| **`__single___wide3`** (ported to firmware) | 68.2 | +18.8 | **1.5 %** | 6.4° |
| `__single__` | 60.6 | +5.3 | 3.6 % | 16° |
| **`flat_tripod`** (the 0.979 level-ground tripod) | 71.9 | +4.9 | **6.8 %** | 31° |
| `__single___wide2` | 65.0 | −14.9 | 6.9 % | 49° |

Two consequences:

1. **The level-ground tripod's 0.979 is partly an artifact.** At 6.8 % clamped commands it is not a
   gait the legs can execute as specified. The tripod-beats-bipod *ordering* is still supported by
   the phase sensitivity above, but that specific number should not be quoted as achievable.
2. **The ported gait is the mildest of the high-lift set** (1.5 %, 6.4°) but not clean. On hardware
   the clamp is the servo calibration, which will not match `jnt_range` exactly.

This also corrects a claim in `README.md`: foot lift is *not* capped at 54 mm. The cap moves with
ride height, and the searched gaits found that coupling unprompted —

| lift | ride 0 | ride +18.8 | ride +25 |
| ---- | ------ | ---------- | -------- |
| 62.5 mm | 7.9 % out | **0.0 %** | 0.0 % |
| 68.2 mm | 9.2 % (33°) | **1.5 % (6.4°)** | 0.0 % |
| 80.0 mm | 10.9 % (45°) | 6.5 % | 3.6 % |

— so `PG_HEIGHT`'s 80 mm ceiling is partly fictional: even with the body fully raised, 80 mm of lift
puts 3.6 % of commands past the femur limit. `optimize_gait_terrain.py --feasible-penalty W`
subtracts `W ×` the out-of-range fraction, so the search can be kept inside what the legs can do.

#### A fully feasible gait costs almost nothing

Re-searching with `--feasible-penalty 3.0` (same cells, commands, budget as `__single___wide3`):

| gait | search score | IK out-of-range | lift | ride | duty | offsets |
| ---- | ------------ | --------------- | ---- | ---- | ---- | ------- |
| `__single___wide3` | 0.912 | 1.6 % | 68 mm | +18.8 | 0.500 | 0, .486, .972, .479, .959, .438 |
| **`__single___feas`** | 0.892 | **0.0 %** | 55 mm | +16.9 | 0.543 | 0, .621, .243, .636, .233, .830 |

Benchmarked paired against `wide3` on identical terrain/command/seed:

| | bench (pooled) | held-out (pooled) | 120 mm | body tilt | body rate |
| --- | -------------- | ----------------- | ------ | --------- | --------- |
| `feas` − `wide3` | −0.007 (p < 0.0001) | **+0.003 (p = 0.36, n.s.)** | +0.020 / +0.028 | **+2.6°** | **+0.29** |

**Eliminating every clamped command is essentially free in score** — indistinguishable on held-out
commands, −0.007 on the search's own command set, and better at the roughest terrain. It is not free
in ride quality: `feas` carries 2.6° more body tilt and 0.29 more body-rate RMS.

Interesting side effect: constrained to feasible commands, the search picked a *different phase
pattern* (`lag_r` 0.621, a tetrapod-ish timing) rather than the near-tripod `wide3` found. The
1-D sensitivity peak at `lag_r` = 0.5 was measured around `wide3`, so it is a local statement; with
the feasibility constraint active the landscape moves.

**Recommendation for hardware:** port `__single___feas`. On the robot the clamp is the servo
calibration, not `jnt_range`, so a gait that relies on clamping is the one sim-to-real assumption
with no evidence behind it — and it costs 0.007 to remove. The swap is one command
(`export_gait.py --key __single___feas`) plus a rebuild. `wide3` remains the better choice if ride
smoothness turns out to matter more than trajectory fidelity once it is on the bench.

### Turning geometry: the stance chord vs the true arc

`stroke = v + omega x r` (commit `c8d2b52`, "twist based foot planning") gives each foot the correct
instantaneous velocity *direction*. But `_stance_curve` then moves the foot in a **straight line**
along that direction, whereas a planted foot's true body-frame path during a turn is an **arc about
the instantaneous centre of rotation**. The gap is not small:

| command | ICR distance | sweep | mid-stance deviation |
| ------- | ------------ | ----- | -------------------- |
| in-place turn, `step_angle` 0.8 | 0 mm | 45.8° | **15.4 mm** |
| fwd 60 mm + turn 0.8 | 75 mm | 45.8° | **19.6 mm** |
| fwd 60 mm + turn 0.4 | 150 mm | 22.9° | 6.4 mm |
| fwd 100 mm + turn 0.2 | 500 mm | 11.5° | 3.4 mm |

`GaitController(arc_stance=True)` sweeps the planted foot along that arc instead, re-anchored so the
endpoints still coincide with the chord (otherwise the swing, which plans a straight line between
them, no longer connects). The residual is the 2.7 % difference between arc length and chord.

**Result — 2×2, gait parameters × stance path, on turn-heavy commands:**

| | chord stance | arc stance | stance effect |
| --- | ------------ | ---------- | ------------- |
| chord-searched gait (`__single___feas`) | 0.616 | 0.628 | +0.012 (p = 0.53, n.s.) |
| arc-searched gait (`__single___arc`) | 0.712 | 0.741 | **+0.029 (p = 0.0006)** |
| *gait effect* | *+0.096 (p = 0.0006)* | *+0.114 (p < 0.0001)* | |

Three conclusions, in order of usefulness:

1. **Correcting the geometry is worth little, and only if you search with it.** Bolting the arc onto
   a chord-tuned gait does nothing (+0.012, n.s.) — its `gyaw`/`pr_yaw` already compensate for the
   chord's corner-cutting, and the arc-searched gait indeed drops both (`gyaw0` 2.38→2.07,
   `pr_yaw` 0.317→0.204). Searched with, the arc is worth +0.029.
2. **The parameters matter 3–4× more than the geometry** (+0.10 vs +0.03), and on the whole-suite
   objective the two gaits are indistinguishable (0.892 vs 0.887). So the arc-searched gait is
   mostly just a *better-turning* gait that the search happened to find.
3. **Which says the search command set under-samples turning.** `COMMANDS_WIDE` is 2 turn commands
   out of 7; the turn-heavy evaluation here is 5 of 6. A gait can be 0.10 better at turning without
   that showing up in the search objective at all. **Weighting the search command set toward turns
   is the higher-value change** — larger effect than the geometry fix, and it costs nothing to try.

Two implementation traps, both caught only because the test included a `fwd_only` command whose
effect must be exactly zero:

- Computing the ICR as `c = (-step_y, step_x)/step_angle` **diverges as the turn rate goes to zero**,
  and `step_angle` is never exactly zero because `yaw_comp` adds a velocity-proportional term to
  straight commands. Subtracting two huge near-equal vectors destroyed the precision and produced a
  large spurious effect on straight-line walking. Reformulated as a perpendicular offset from the
  chord whose scalar factor is O(`ang`), so straight walking is untouched by construction.
- That reformulation returns the *offset* from the chord, but the call site still **overwrote**
  `delta` rather than adding to it, deleting the stride itself. Every arm collapsed (−0.47 pooled)
  before this was spotted.

Still unexamined in the gait engine: the stance depth curve uses `depth·cos(pi(x+y)/(2·length))`,
whose mixing of x and y makes it direction-dependent in a way that looks unintended; swing is planar
along the chord direction, so touchdown inherits the same (smaller, non-slipping) error; and the
default stance polygon does not rotate into a turn.

### Porting to firmware

**Done (builds clean on `esp32-wroom-camera`, RAM 29.2 %, flash 20.5 %):**

| change | where |
| ------ | ----- |
| `GaitType::TUNED` appended (existing wire values unchanged) | `message_types.h` |
| `setGait()` case loading the searched offsets + duty | `gait.h` |
| `gait_state_t.phase_rate` — explicit cadence, 0 = legacy law | `gait.h` |
| WALK branch: velocity command → `velocity_to_gait` → stride/lift/cadence/ride | `motion.h` |
| generated constants | `firmware/include/gait_tuned.h` ← `simulation/export_gait.py` |
| drift guard | `simulation/test_firmware_gait_parity.py` |

Three things the port had to confront that the sim study had not:

1. **Foot lift could not even be expressed.** The WALK branch computes `step_height = (s1+1)*20`,
   i.e. 0–40 mm from a slider, defaulting to 15 mm. The tuned gait needs 68 mm. (Note this also
   corrects a claim made earlier in this document: the 11.7 mm figure is `gait_coef.json`'s choice
   in the *sim's* analytic map, not a firmware limit — the firmware's limit is 40 mm.) `TUNED` now
   fixes lift at the searched value and lets `s1` trim ±15 mm around it.
2. **The cadence laws disagree.** The firmware advances phase by
   `step_speed × clip(max(|length|/25, |angle|×1.5), 0.75, 1.5)`; the searched gaits use
   `pr_base + pr_slope·speed + pr_yaw·|yaw|`. Measured divergence: 0.90–1.24× when walking forward,
   but **1.80× for an in-place turn**, where the legacy `clip` floors cadence at 0.75 cyc/s and the
   tuned gait wants 1.35. Stride and cadence were co-optimized, so porting one without the other
   reproduces neither.
3. **The stick means something different.** For `TUNED` it is a *velocity* command over the
   envelope the search covered (±0.45 m/s forward, ±0.12 lateral, ±1.0 rad/s), because that is the
   input the gains were fitted against. The legacy gait types keep the old direct-stride mapping.

**Servo feasibility, checked before any of this** (`servo_feasibility.py`, `servo_margin.py`):

| arm | torque mean | at limit | joint speed p99 | lag mean | score at ×0.5 stall |
| --- | ----------- | -------- | --------------- | -------- | ------------------- |
| firmware gait | 0.68 | 39.8 % | 0.35 | 0.75° | 0.88 of nominal |
| `__single___wide3` | 0.71 | 42.8 % | 0.74 | 6.22° | **0.93 of nominal** |
| `sched_v5` | 0.73 | 46.3 % | 0.76 | 5.77° | 1.15 of nominal |

The tuned gaits carry ~8× the servo tracking lag of the shipped gait, which looked alarming until
the margin sweep: at **half** the modelled stall torque `wide3` still retains 93 % of its score —
degrading more gracefully than the shipped gait — with zero falls in 1512 episodes. Joint speed is
never near the no-load cap. The lag is the cost of a bigger, faster stride, not a sign of running
out of servo.

**Not ported:** the policy. `export_policy.py` cannot emit a `GaitSchedule` base or the 27-D
`residual_sched` layout, so `sched_v5` has no deploy path yet. Given it only reaches parity with
this gait, the gait is the thing worth shipping first.

**Untested on hardware.** Everything above is a sim result plus a clean compile. The first real-robot
checks should be: foot lift actually reaching ~68 mm without the femur saturating, current draw at
duty 0.50 with a 1.9 mm stance push, and whether the velocity-command stick mapping feels usable.

### What to change on the robot

Independent of the RL question, the search produced findings that are worth applying to the
firmware directly, since they need no policy at all:

1. **Raise the foot lift.** `gait.h` asks for ~11.7 mm; every terrain in the suite wants 26–76 mm,
   and the single best all-round value is ~55–61 mm (72 mm for a level-ground tripod). This is the
   single highest-value change.
1b. **If you keep the discrete gait types, use these level-ground settings** (`flat_tripod` /
   `flat_bipod` in the library): tripod `stand_frac` 0.608 with 72 mm lift; bipod `stand_frac` 0.749
   with 62 mm lift. Note the optimized bipod is no longer a 2-legs-down gait — at duty 0.749 it
   keeps ~4.5 legs down, so `BI_STAND_FRAC = 2.1/6` is the wrong constant to keep.
2. **`pr_yaw`: evidence is mixed, do not treat ≈1.3 as settled.** Three wide-command searches
   returned 1.31, 1.33 and **0.19**, the last being the best-resourced one (`__single___wide3`,
   1280 evaluations) — and it has the *best* yaw tracking of the three (0.036 rad/s). Turn-rate
   tracking is evidently reachable either by boosting turn cadence (`pr_yaw` ≈ 1.3, `gyaw0` ≈ 3.4)
   or by a larger step angle at normal cadence (`pr_yaw` 0.19, `gyaw0` 2.27). The sim's current
   0.109 sits at neither optimum; which branch to adopt should be decided on hardware, where servo
   rate limits break the tie.
3. **Raise the duty factor to ≈0.63.** This one needs a `gait.h` change, not just a coefficient:
   the shipped tripod/bipod patterns top out at 0.517.
4. Do **not** build a terrain classifier, and do not add wave/ripple gaits.

#### Is terrain knowledge worth anything? (paired, same seeds)

| arm | flat | 40 mm | 80 mm | 120 mm |
| --- | ---- | ----- | ----- | ------ |
| `tuned_single` | 0.94 | 0.93 | 0.90 | 0.75 |
| `tuned_oracle` (told the terrain) | 0.95 | 0.94 | 0.89 | 0.77 |

Confirmed on identical terrain seeds: **+0.01 at best, and negative at 80 mm.** A per-terrain gait
library is not worth building for this robot; the oracle also pays for its specialization with more
tilt (10.6° vs 8.6° at 120 mm) and more knock (0.128 vs 0.081).

### Optimizer comparison

Same two conditions, same 640-evaluation budget, same search seeds (0–3), same held-out seeds
(500–507), same four free seed points, same 12 workers.

| optimizer | bumps 80 search → held out | steps 120 search → held out | evals used | wall time |
| --------- | -------------------------- | --------------------------- | ---------- | --------- |
| **CMA-ES** | 0.950 → **0.953** | 0.902 → **0.853** | 640 | 333 s / 303 s |
| Optuna TPE (Bayesian) | **0.956** → 0.920 | 0.847 → 0.758 | 640 | 330 s / 334 s |
| Differential evolution | 0.929 → 0.924 | 0.824 → 0.781 | 1152 | 453 s / 458 s |

CMA-ES wins on held-out score in both conditions, which is the number that counts.

The instructive row is TPE on `bumps 80`: it found the **best search score of any optimizer**
(0.956) and then generalized worst of the three (0.920). That is overfitting to the four search
seeds — exactly what the held-out protocol exists to catch, and a reason not to report search
scores as results. CMA-ES, by contrast, generalized slightly *upward* (0.950 → 0.953).

DE is listed for completeness but had an unfair advantage and still lost: scipy's generation size is
`popsize × D`, so even at `popsize=1` it runs 18 individuals per generation and overshot the budget
to 1152 evaluations (1.8×).

This is the regime where a Bayesian surrogate is not expected to pay: 18 continuous dimensions, a
noisy objective, and evaluations cheap enough (~0.5 s per candidate on 12 workers) that thousands
are affordable. Surrogate modelling earns its keep when evaluations are expensive; here the model
overhead buys nothing and the density-ratio surrogate latches onto seed noise. CMA-ES is the
default for that reason, and `--optimizer tpe` remains available so the claim can be re-tested when
the objective or the budget changes.

### The gait library (per-terrain oracle)

CMA-ES, 640 evaluations per condition, held out on seeds 500–507. `lift` is the commanded foot
lift in mm (**firmware: 11.7 mm**), `ride` the body-height offset in mm.

| terrain | score | firmware tripod | prog | stuck | duty | lift | ride | offsets |
| ------- | ----- | --------------- | ---- | ----- | ---- | ---- | ---- | ------- |
| `__single__` | 0.859 | 0.243 | 0.98 | 0.11 | 0.63 | 60.6 | +5.3 | 0, .42, .84, .50, .94, .37 |
| flat | 0.954 | 0.552 | 1.00 | 0.06 | 0.81 | 65.4 | −9.5 | 0, .34, .67, .42, .89, .36 |
| bumps 40 | 0.936 | 0.436 | 1.07 | 0.05 | 0.76 | 57.1 | −4.5 | 0, .56, .11, .40, .02, .64 |
| rocks 40 | 0.966 | 0.355 | 1.05 | 0.04 | 0.47 | 38.2 | +13.0 | 0, .44, .87, .47, .92, .38 |
| steps 40 | 0.939 | 0.399 | 1.05 | 0.04 | 0.61 | 25.9 | −12.0 | 0, .43, .86, .64, .16, .68 |
| waves 40 | 0.940 | 0.324 | 1.06 | 0.07 | 0.67 | 30.5 | +14.7 | 0, .52, .04, .32, .86, .39 |
| bumps 80 | 0.953 | 0.189 | 1.06 | 0.05 | 0.50 | 52.5 | +0.8 | 0, .50, .01, .46, .02, .58 |
| rocks 80 | 0.887 | 0.184 | 1.10 | 0.11 | 0.56 | 57.6 | +9.0 | 0, .45, .90, .38, .80, .21 |
| steps 80 | 0.872 | 0.184 | 0.96 | 0.15 | 0.64 | 66.7 | +0.8 | 0, .50, 1.0, .10, .72, .35 |
| waves 80 | 0.847 | 0.123 | 1.04 | 0.17 | 0.67 | 67.4 | −0.2 | 0, .48, .97, .94, .52, .10 |
| bumps 120 | 0.822 | 0.108 | 1.11 | 0.13 | 0.64 | 75.8 | +8.3 | 0, .56, .13, .42, .96, .50 |
| rocks 120 | 0.596 | 0.123 | 0.70 | 0.32 | 0.60 | 60.5 | +5.9 | 0, .62, .25, .24, .93, .62 |
| steps 120 | 0.853 | 0.120 | 0.87 | 0.14 | 0.58 | 60.0 | +15.3 | 0, .60, .20, .60, .16, .72 |
| waves 120 | 0.739 | 0.084 | 0.89 | 0.26 | 0.55 | 48.9 | +8.6 | 0, .51, .02, .70, .39, .09 |

Four things this says:

1. **Foot lift is the dominant parameter, and the firmware's is 3–6× too small.** Every terrain
   wants 26–76 mm; the firmware asks for 11.7 mm. Nothing else in the vector comes close to
   mattering as much.
2. **Duty factor lands at 0.47–0.81, mostly 0.55–0.67 — outside what the old policy could ask
   for.** `residual_gait` spans 0.35–0.517, so 11 of these 14 gaits are unreachable for it. This is
   the measurement that justifies `residual_sched`, and it is not a small effect.
3. **Leg phasing barely matters.** The winners cluster around tripod and metachronal-tetrapod
   patterns; `bumps 80` converged to an essentially ideal tripod (0, .50, .01, .46, .02, .58).
   Wave and ripple were available and were rejected everywhere. Ride height wanders between
   −12 and +15 mm with no trend, i.e. it is close to irrelevant on rough fields (the curb is a
   different story — see below).
4. **Terrain knowledge is worth almost nothing.** Mean oracle score over the 13 conditions is
   **0.870** against **0.859** for the one gait that has to serve all of them — about +0.011.
   The robot does not need a terrain classifier; it needs one properly chosen gait. The exception
   is `rocks 120` (0.596), the only condition where even a specialized gait struggles.

### Best tripod and best bipod on level ground

The library's `flat` entry lets the search choose the phase pattern, and it chooses neither tripod
nor bipod. That answers "what is the best gait?" but not "what is the best *tripod*?" — which is the
question that matters for the firmware, where tripod and bipod are discrete `GaitType` values.

`--fix-pattern {tripod,bipod}` pins `lag_r`/`lag_l`/`contra` and searches the remaining 15
parameters. Level ground, CMA-ES, 640 evaluations, held out on seeds 500–507:

| | score | progress | stuck | **duty** | **foot lift** | ride | tilt | knock |
| --- | ----- | -------- | ----- | -------- | ------------- | ---- | ---- | ----- |
| **best tripod** | **0.979** | 1.00 | 0.02 | **0.608** | **71.9 mm** | +4.9 | 0.4° | 0.002 |
| **best bipod** | 0.953 | 1.09 | 0.03 | **0.749** | **62.3 mm** | −6.3 | 1.3° | 0.000 |
| firmware tripod | 0.552 | — | — | 0.517 | 11.7 mm | 0 | — | — |
| firmware bipod | 0.906 | — | — | 0.350 | 11.7 mm | 0 | — | — |

Full parameter sets are in `gait_library.json` under `flat_tripod` / `flat_bipod`.

| | `gx0` | `gx1` | `gy0` | `gy1` | `gyaw0` | `gyaw1` | `blend_speed` | `pr_base` | `pr_slope` | `pr_yaw` | `step_depth` |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| tripod | 0.341 | 0.197 | 0.562 | 0.376 | 1.621 | 3.024 | 0.249 | −0.165 | 0.602 | 0.696 | 1.30 |
| bipod | 0.444 | 0.184 | 0.600 | 0.194 | 3.111 | 2.585 | 0.315 | +0.176 | 0.891 | 1.635 | 4.06 |

Three things worth noting:

1. **On level ground the tripod is the better gait** (0.979 vs 0.953), and it is essentially perfect
   — progress 1.00, stalled 2 % of the time, 0.4° of body tilt. The intuition that a faster, more
   dynamic bipod should win on clean ground does not hold here.
2. **Both want far more duty than the firmware gives them**: 0.608 against 0.517 for tripod, and
   0.749 against 0.350 for bipod. The optimized "bipod" is really a high-duty tetrapod-timed gait —
   it keeps ~4.5 legs down. And both want 6× the foot lift.
3. **The optimized tripod beat the unconstrained 18-parameter search** (0.979 vs 0.954) — which is
   impossible for a genuine optimum over a superset, and so was a convergence artifact: the pinned
   searches spent the same 640 evaluations on 15 dimensions instead of 18. Confirmed by re-running
   the unconstrained flat case at 2400 evaluations:

| search | evals | dims | score | duty | lift | offsets |
| ------ | ----- | ---- | ----- | ---- | ---- | ------- |
| unconstrained | **2400** | 18 | **0.996** | 0.779 | 59.5 mm | 0, .51, .02, .27, .77, .28 |
| tripod (pinned) | 640 | 15 | 0.979 | 0.608 | 71.9 mm | 0, .50, 0, .50, 0, .50 |
| bipod (pinned) | 640 | 15 | 0.953 | 0.749 | 62.3 mm | 0, .33, .67, .67, .33, 0 |
| unconstrained | 640 | 18 | 0.954 | 0.807 | 65.4 mm | 0, .34, .67, .42, .89, .36 |

The ordering is restored, and the deep winner is an *asymmetric* gait: tripod-timed on the right
(0, .51, .02) with the left side shifted off it (.27, .77, .28).

**Consequence for everything above: every library entry searched at 640 evaluations is a lower
bound, and the flat case shows the deficit can be ≈ 0.04.** This does not threaten the
policy-vs-gait conclusion — more budget only makes the gaits better — but it does weaken the
oracle-vs-single margin (+0.011), since both arms are under-converged by a comparable and unmeasured
amount. Re-running the full library at 2400 evaluations is the obvious next step and was not done
here.

### Obstacle clearance

The curb fixture is a single full-width step 0.6 m ahead, approached at 0.20 m/s for 14 s. "Cleared"
means reaching 0.15 m past the edge and still going.

Two defects in the original fixture had to be fixed before it measured anything:

1. **The objective saturated.** `curb_score` capped at 1.0 once the step was cleared, so every
   candidate that got over scored exactly 1.0 and the search had no gradient — it returned whichever
   candidate cleared first, with no preference for clearing it *well*. It now subtracts the same
   knock and rate terms as the field score, which discriminates among clearing gaits.
2. **The ceiling was the model's, not the robot's.** The heightfield declared `zmax = 0.15 m`, so
   the sweep asserted out at 0.16 m. Since `randomize_hfield` writes `h·(amp/zmax)`, the realized
   height is `amp` regardless of `zmax` — verified directly (0.08 m requested → 0.0796 m realized at
   both settings) — so raising it to 0.30 m is physically neutral for every result above and simply
   lets the sweep find where the robot actually fails.

CMA-ES per curb height, 512 evaluations, held out on 8 seeds. `best_seed` is the best of the four
named gaits (tripod / bipod / wave / ripple on the tuned coefficients) at that height.

| curb | score | best named gait | reach (m) | cleared | duty | lift (mm) | knock |
| ---- | ----- | --------------- | --------- | ------- | ---- | --------- | ----- |
| 40 mm | 1.000 | 0.888 | 0.76 | yes | 0.50 | 19.9 | 0.000 |
| 60 mm | 1.000 | 0.779 | 0.82 | yes | 0.60 | 35.5 | 0.000 |
| 80 mm | 1.000 | 0.776 | 1.36 | yes | 0.71 | 52.7 | 0.000 |
| 100 mm | 1.000 | 0.592 | 0.76 | yes | 0.56 | 59.2 | 0.001 |
| 120 mm | 0.993 | 0.594 | 1.63 | yes | 0.61 | 54.3 | 0.027 |
| 140 mm | 0.985 | 0.586 | 1.46 | yes | 0.59 | 72.2 | 0.060 |
| 160 mm | 0.886 | 0.594 | 1.56 | yes | 0.55 | 56.9 | **0.455** |
| 180 mm | 0.789 | 0.589 | 0.60 | **no** | 0.68 | 32.9 | 0.066 |
| 200 mm | 0.757 | 0.594 | 0.57 | **no** | 0.68 | 54.9 | 0.028 |
| 240 mm | 0.711 | 0.590 | 0.53 | **no** | 0.88 | 31.5 | 0.007 |

Reading the knock column matters here. Up to **140 mm** the gait steps over cleanly (knock ≤ 0.06).
At **160 mm** it still gets over, but with knock 0.455 — it is scrambling, dragging shins and belly
across the edge, which is not something to promise on hardware. At **180 mm and above** it stops at
the edge (reach 0.53–0.60 m against an edge at 0.60 m) regardless of parameters.

So, against the previously documented ceilings of 30 mm (firmware gait), 80 mm (trained policy) and
100 mm (policy + contact reflexes):

> **Clean clearance ceiling: 140 mm. Scrambling ceiling: 160 mm. Hard failure at 180 mm.**

140 mm is 2.1× the robot's 66 mm stance height, and it comes from tuning constants, not from
learning or from extra sensing. Note also that above 100 mm no named gait reaches the edge at all
within the 14 s window (`best_seed` ≈ 0.59, i.e. ~0.44 m of travel) — the firmware-family gaits are
not slow *at* the obstacle, they are too slow to get there.
