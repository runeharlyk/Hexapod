"""The firmware's heading hold (firmware/include/heading_hold.h) walking the MuJoCo robot: the bipod drifts in yaw as its
feet slip, and the hold, steering on the fused odometry's yaw, keeps the heading. Measured on 2026-11-04: a full forward
stick on BI turned -8.4 deg in 8 s unheld and -0.5 deg held; TRI and WAVE drift under 0.5 deg there, inside the hold's
deadband."""
import math

from src.robot import firmware_gait as fg
from src.sim.firmware_loop import FIRMWARE_DT, FirmwareLoop
from src.sim.mj_runtime import HexapodSim


def _true_yaw(sim: HexapodSim) -> float:
    w, x, y, z = sim.base_quat()
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _run(loop: FirmwareLoop, seconds: float) -> None:
    for _ in range(round(seconds / FIRMWARE_DT)):
        loop.tick()


def _turned_walking(hold: bool, seconds: float = 8.0) -> float:
    """The base's true yaw, in degrees, turned over `seconds` of a full forward stick on BI, after a second's start."""
    sim = HexapodSim()
    sim.reset_to_stand()
    loop = FirmwareLoop(sim, fg.BI_GATE)
    loop.hold()
    _run(loop, 1.0)
    loop.walk(lx=0.0, ly=1.0, rx=0.0, hold_heading=hold)
    _run(loop, 1.0)
    turned, last = 0.0, _true_yaw(sim)
    for _ in range(round(seconds / FIRMWARE_DT)):
        loop.tick()
        now = _true_yaw(sim)
        turned += math.remainder(now - last, 2 * math.pi)
        last = now
    return math.degrees(turned)


def test_the_hold_keeps_a_drifting_bipod_on_its_heading():
    unheld = _turned_walking(hold=False)
    held = _turned_walking(hold=True)
    assert abs(unheld) > 4.0, unheld
    assert abs(held) < 1.5, held
    assert abs(held) < abs(unheld) / 4
