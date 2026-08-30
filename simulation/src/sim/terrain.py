"""Per-episode heightfield generation for uneven-terrain training/eval.

Requires the hfield model variant (`build_model.py --terrain` -> model_terrain.xml).
Every generator returns a normalized field in [0, 1] that is scaled by `max_height`,
with a flat disk at the origin so the robot always spawns cleanly.

Kinds:
  bumps  smooth random hills          (feature = spatial frequency)
  rocks  scattered flat-topped blocks (feature = density)   -- step-over obstacles
  steps  tiled plateaus               (feature = tiles/m)   -- discrete footholds, sharp edges
  waves  sinusoidal rolling ground    (feature = waves/m)   -- sustained up/down grades
  mixed  one of the above, sampled per episode (training)

Equal nominal height is not equal difficulty -- a sharp-edged step is far harder than a smooth
hill of the same amplitude -- so each kind carries an amplitude factor that makes one curriculum
`max_height` roughly comparable across kinds.
"""

import mujoco
import numpy as np

FLAT_RADIUS = 0.20   # m, flat spawn disk around the origin (feet stance radius ~0.16 m)
RAMP = 0.15          # m, smoothstep ramp from flat disk to full terrain
COARSE_N = 33        # coarse noise grid -> ~0.31 m feature wavelength on a 10 m field

KINDS = ("bumps", "rocks", "steps", "waves", "curb")
MIX_KINDS = ("bumps", "rocks", "steps", "waves")   # 'curb' is a measurement fixture, not training ground
MIX_WEIGHTS = (0.40, 0.25, 0.20, 0.15)             # bumps dominate: the generic rough-ground case
AMPLITUDE = {"bumps": 1.0, "rocks": 0.85, "steps": 0.7, "waves": 0.9, "curb": 1.0}

CURB_AT = 0.6        # m ahead of the spawn (+Y = forward) where the curb edge sits
CURB_DEPTH = 1.0     # m of raised ground beyond the edge


def _upsample_bilinear(coarse, n):
    k = coarse.shape[0]
    x = np.linspace(0.0, k - 1.0, n)
    xi = np.floor(x).astype(int)
    xf = (x - xi)[:, None]
    xi1 = np.minimum(xi + 1, k - 1)
    rows = coarse[xi] * (1.0 - xf) + coarse[xi1] * xf
    return rows[:, xi] * (1.0 - xf.T) + rows[:, xi1] * xf.T


def _bumps(nrow, rng, feature):
    coarse = max(4, int(round(COARSE_N * feature)))  # higher feature -> finer, bumpier
    return _upsample_bilinear(rng.uniform(0.0, 1.0, (coarse, coarse)), nrow)


def _rocks(nrow, rng, feature):
    """Scattered flat-topped blocks with gaps between them -- step-over obstacles."""
    h = np.zeros((nrow, nrow))
    n = max(1, int(400 * feature))         # dense field; more/closer rocks with higher 'feature'
    for _ in range(n):
        cx, cy = int(rng.integers(0, nrow)), int(rng.integers(0, nrow))
        w = int(rng.integers(2, 8))        # block half-size (cells; ~40 mm each)
        ht = float(rng.uniform(0.35, 1.0))
        h[max(0, cx - w):cx + w, max(0, cy - w):cy + w] = ht
    return h


def _steps(nrow, rng, feature, span_m):
    """Tiled plateaus of random height: every tile is flat with sharp edges between neighbours,
    so the robot must place feet on discrete levels instead of a continuous surface."""
    tile_m = np.clip(0.35 / max(feature, 0.25), 0.12, 1.2)  # m per tile
    ntile = max(2, int(round(span_m / tile_m)))
    tiles = rng.uniform(0.0, 1.0, (ntile, ntile))
    reps = int(np.ceil(nrow / ntile))
    return np.kron(tiles, np.ones((reps, reps)))[:nrow, :nrow]


def _waves(nrow, rng, feature, span_m):
    """Sinusoidal rolling ground in a random direction: sustained grades rather than isolated
    bumps, which is what actually challenges body-attitude control."""
    lam = np.clip(1.2 / max(feature, 0.25), 0.35, 4.0)      # m per wave
    k = 2.0 * np.pi / lam
    ang = float(rng.uniform(0.0, 2.0 * np.pi))
    coords = np.linspace(-span_m / 2, span_m / 2, nrow)
    yy, xx = np.meshgrid(coords, coords, indexing="ij")
    phase = k * (np.cos(ang) * xx + np.sin(ang) * yy) + rng.uniform(0.0, 2.0 * np.pi)
    return 0.5 * (1.0 + np.sin(phase))


def _curb(nrow, span_m):
    """A single full-width step edge `CURB_AT` m ahead (+Y): the cleanest measurement of how tall an
    obstacle the robot can actually climb, as opposed to how well it copes with a rough field."""
    coords = np.linspace(-span_m / 2, span_m / 2, nrow)
    band = (coords >= CURB_AT) & (coords <= CURB_AT + CURB_DEPTH)
    return np.tile(band[:, None].astype(float), (1, nrow))  # rows index y (forward)


def sample_kind(rng):
    """Pick a terrain kind for one episode (training with kind='mixed')."""
    return str(rng.choice(MIX_KINDS, p=MIX_WEIGHTS))


def randomize_hfield(model, rng, max_height, kind="bumps", feature=1.0):
    """Fill the 'terrain' hfield with `kind`, peak amplitude `max_height` m. Returns the kind used
    (resolves 'mixed'). A flat spawn disk at the origin keeps every reset well-conditioned."""
    if kind == "mixed":
        kind = sample_kind(rng)
    hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "terrain")
    nrow, ncol = int(model.hfield_nrow[hid]), int(model.hfield_ncol[hid])
    rx, _, zmax, _ = model.hfield_size[hid]
    amp = max_height * AMPLITUDE[kind]
    assert amp <= zmax, f"terrain {amp} m exceeds hfield zmax {zmax} m"

    if kind == "rocks":
        h = _rocks(nrow, rng, feature)
    elif kind == "steps":
        h = _steps(nrow, rng, feature, 2.0 * rx)
    elif kind == "waves":
        h = _waves(nrow, rng, feature, 2.0 * rx)
    elif kind == "curb":
        h = _curb(nrow, 2.0 * rx)
    else:
        h = _bumps(nrow, rng, feature)

    if kind != "curb":  # the curb starts beyond the disk; fading it would round off the edge
        coords = np.linspace(-rx, rx, nrow)  # flat spawn disk, smoothstep out to full amplitude
        dist = np.hypot(coords[:, None], coords[None, :])
        t = np.clip((dist - FLAT_RADIUS) / RAMP, 0.0, 1.0)
        h = h * (t * t * (3.0 - 2.0 * t))

    adr = model.hfield_adr[hid]
    model.hfield_data[adr:adr + nrow * ncol] = (h * (amp / zmax)).ravel()
    return kind
