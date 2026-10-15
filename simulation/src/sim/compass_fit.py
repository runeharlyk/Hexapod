"""Hard-iron calibration of the compass from a telemetry recording made while the robot is turned through orientations.

The recording holds the readings as the chip gave them, before the stored offset (TickSample.mag), so a robot that was
calibrated before is fitted afresh, not on top of the old offset. A sphere fitted to the readings has the hard-iron
offset as its centre and the local field strength as its radius.

The heading uses only the chip's x and y, and the Earth's field points steeply downward (about 70 degrees in Denmark),
so a robot that is only yawed leaves the readings on one horizontal circle: x and y are determined, z and the radius
are not. Each centre axis therefore carries its standard error, and the fit says which axes it can vouch for.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.sim.recording import Recording

MIN_SAMPLES = 10
# 0.5 uT of offset error is under 2 degrees of heading even where the horizontal field is only 18 uT.
MAX_STD_ERROR_UT = 0.5
_REFINEMENTS = 20


@dataclass
class CompassFit:
    offset: np.ndarray  # uT, chip frame; z is 0 when it is not determined
    std_error: np.ndarray  # uT, per centre axis
    radius: float  # uT, the field's strength once the offset is removed; meaningful only when z is determined
    horizontal_radius: float  # uT, mean distance of the readings from the centre in the chip's xy plane
    residual: float  # uT, RMS distance of the readings from the sphere
    samples: int

    @property
    def xy_determined(self) -> bool:
        return bool(np.all(self.std_error[:2] <= MAX_STD_ERROR_UT))

    @property
    def z_determined(self) -> bool:
        return self.xy_determined and bool(self.std_error[2] <= MAX_STD_ERROR_UT)


def raw_readings(rec: Recording) -> np.ndarray:
    """One reading per chip sample: ticks that did not read the compass are skipped, and the repeats of a reading
    that the control loop polled again before the 15 Hz chip produced a new one are dropped."""
    mag = np.asarray(rec.ticks["mag"], dtype=float)
    if mag.ndim != 2 or mag.shape[1] != 3:
        return np.empty((0, 3))  # a recording from a firmware that does not stream the compass
    mag = mag[np.asarray(rec.ticks["mag_fresh"], dtype=bool)]
    if len(mag) == 0:
        return mag
    changed = np.any(mag[1:] != mag[:-1], axis=1)
    return mag[np.concatenate([[True], changed])]


def _jacobian(points: np.ndarray, centre: np.ndarray):
    offsets = points - centre
    distances = np.linalg.norm(offsets, axis=1)
    return distances, np.hstack([-offsets / distances[:, None], -np.ones((len(points), 1))])


def fit_sphere(points: np.ndarray) -> CompassFit:
    """Least-squares sphere: the algebraic solution, refined to minimise the distance of the points from it."""
    if len(points) < MIN_SAMPLES:
        raise ValueError(f"{len(points)} compass samples; turn the robot through every orientation while recording")
    design = np.hstack([2 * points, np.ones((len(points), 1))])
    solution, *_ = np.linalg.lstsq(design, np.sum(points**2, axis=1), rcond=None)
    centre = solution[:3]
    radius = float(np.sqrt(solution[3] + centre @ centre))
    for _ in range(_REFINEMENTS):
        distances, jacobian = _jacobian(points, centre)
        step, *_ = np.linalg.lstsq(jacobian, radius - distances, rcond=None)
        centre, radius = centre + step[:3], radius + float(step[3])
        if np.max(np.abs(step)) < 1e-9:
            break
    distances, jacobian = _jacobian(points, centre)
    errors = distances - radius
    variance = errors @ errors / (len(points) - 4)
    std_error = np.sqrt(np.diag(variance * np.linalg.pinv(jacobian.T @ jacobian)))[:3]
    horizontal = float(np.mean(np.hypot(*(points[:, :2] - centre[:2]).T)))
    fit = CompassFit(centre, std_error, radius, horizontal, float(np.sqrt(np.mean(errors**2))), len(points))
    if not fit.z_determined:
        fit.offset = np.array([centre[0], centre[1], 0.0])
    return fit


def fit_recording(rec: Recording) -> CompassFit:
    return fit_sphere(raw_readings(rec))
