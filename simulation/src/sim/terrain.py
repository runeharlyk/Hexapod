"""Per-episode heightfield randomization for uneven-terrain training/eval.

Requires the hfield model variant (`build_model.py --terrain` -> model_terrain.xml).
Generates smooth random bumps in [0, max_height] m by bilinear-upsampling coarse
uniform noise, with a flat disk at the origin so the robot always spawns cleanly.
The policy is blind (IMU only, no exteroception), so terrain must stay gentle
relative to the ~66 mm body clearance and 20-50 mm step height.
"""

import mujoco
import numpy as np

FLAT_RADIUS = 0.20   # m, flat spawn disk around the origin (feet stance radius ~0.16 m)
RAMP = 0.15          # m, smoothstep ramp from flat disk to full terrain
COARSE_N = 33        # coarse noise grid -> ~0.31 m feature wavelength on a 10 m field


def _upsample_bilinear(coarse, n):
    k = coarse.shape[0]
    x = np.linspace(0.0, k - 1.0, n)
    xi = np.floor(x).astype(int)
    xf = (x - xi)[:, None]
    xi1 = np.minimum(xi + 1, k - 1)
    rows = coarse[xi] * (1.0 - xf) + coarse[xi1] * xf
    return rows[:, xi] * (1.0 - xf.T) + rows[:, xi1] * xf.T


def _rocks(nrow, rng, feature):
    """Scattered flat-topped blocks (rocks) with gaps between them -- step-over obstacles."""
    h = np.zeros((nrow, nrow))
    n = max(1, int(400 * feature))         # dense field; more/closer rocks with higher 'feature'
    for _ in range(n):
        cx, cy = int(rng.integers(0, nrow)), int(rng.integers(0, nrow))
        w = int(rng.integers(2, 8))        # block half-size (cells; ~40 mm each)
        ht = float(rng.uniform(0.35, 1.0))
        h[max(0, cx - w):cx + w, max(0, cy - w):cy + w] = ht
    return h


def _slope(nrow):
    """Uphill ramp: flat behind the origin (x<0), rising linearly toward +x (forward)."""
    xs = np.linspace(-1.0, 1.0, nrow)      # normalized x across the field (col index)
    ramp = np.clip(xs, 0.0, 1.0)
    return np.tile(ramp, (nrow, 1))        # h[y, x]


def randomize_hfield(model, rng, max_height, kind="bumps", feature=1.0):
    """Fill the 'terrain' hfield to amplitude [0, max_height] m.

    kind: "bumps" (smooth random hills; feature scales bumpiness/frequency), "rocks" (discrete
    step-over blocks), or "slope" (uphill ramp toward +x). Default matches training exactly.
    """
    hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "terrain")
    nrow, ncol = int(model.hfield_nrow[hid]), int(model.hfield_ncol[hid])
    rx, _, zmax, _ = model.hfield_size[hid]
    assert max_height <= zmax, f"terrain {max_height} m exceeds hfield zmax {zmax} m"

    if kind == "slope":
        h = _slope(nrow)
        disk = False                        # origin is already level (h=0 at center)
    elif kind == "rocks":
        h = _rocks(nrow, rng, feature)
        disk = True
    else:  # bumps
        coarse = max(4, int(round(COARSE_N * feature)))  # higher feature -> finer, bumpier
        h = _upsample_bilinear(rng.uniform(0.0, 1.0, (coarse, coarse)), nrow)
        disk = True

    if disk:  # flat spawn disk with a smoothstep ramp out to full amplitude
        coords = np.linspace(-rx, rx, nrow)
        dist = np.hypot(coords[:, None], coords[None, :])
        t = np.clip((dist - FLAT_RADIUS) / RAMP, 0.0, 1.0)
        h = h * (t * t * (3.0 - 2.0 * t))

    adr = model.hfield_adr[hid]
    model.hfield_data[adr:adr + nrow * ncol] = (h * (max_height / zmax)).ravel()
