"""The firmware's climb walk (firmware/include/gait_climb.h, export_climb.py) against a sharp-edged step in MuJoCo: a box
whose vertical face stands across the robot's path, as a stack of books or a kerb does, unlike the terrain heightfield's
curb, whose 39 mm cells ramp the edge. Where the feet meet the edge decides a single attempt, so each walk starts at
START_DISTANCES from the face, spread over a stride. Measured on 2026-11-08 on WAVE over 60 s, the classic walk at its
tallest lift on a raised body (s1 1, h -0.5) got onto a 60 mm step from 2 of the 6 distances and a 70 mm one from 1 at
the default speed and 2 at full speed; the climb, an 80 mm lift on a body raised 37.5 mm, from all 6 at both speeds,
and onto 80 mm from 1."""
import os

import numpy as np

from src.robot import firmware_gait as fg
from src.sim.firmware_loop import FIRMWARE_DT, FirmwareLoop
from src.sim.mj_runtime import MODEL_PATH, HexapodSim

START_DISTANCES = np.linspace(0.40, 0.52, 6)  # m from the spawn to the step's face
STEP_DEPTH = 3.0  # m of raised floor beyond the face, more than any walk here covers
FLOOR_S = 5.0  # s of walking on the floor, once the posture is taken and before a foot reaches the face
SECONDS = 60.0


def step_model(height_mm: float, face_at: float, directory) -> str:
    """The robot's model with a box step `height_mm` tall across its path, its face `face_at` m ahead, written into
    `directory`."""
    with open(MODEL_PATH) as f:
        xml = f.read()
    half = height_mm / 2000
    box = (f'<geom name="step" type="box" pos="0 {face_at + STEP_DEPTH / 2} {half}" size="1 {STEP_DEPTH / 2} {half}" '
           'condim="3" friction="1.0 0.02 0.001" contype="1" conaffinity="6"/>')
    xml = xml.replace("<worldbody>", "<worldbody>\n" + box, 1)
    xml = xml.replace('meshdir="stl"', f'meshdir="{os.path.dirname(os.path.abspath(MODEL_PATH))}/stl"')
    path = os.path.join(directory, "step.xml")
    with open(path, "w") as f:
        f.write(xml)
    return path


def walked_onto(height_mm: float, face_at: float, directory, **walk) -> bool:
    """Whether a forward stick on WAVE carries the body onto the step within SECONDS: past the face, and risen by most
    of the step above the height it walked at on the floor after FLOOR_S."""
    sim = HexapodSim(step_model(height_mm, face_at, directory))
    sim.reset_to_stand()
    loop = FirmwareLoop(sim, fg.WAVE)
    loop.hold()
    for _ in range(round(1.0 / FIRMWARE_DT)):
        loop.tick()
    y0 = sim.data.qpos[1]
    loop.walk(lx=0.0, ly=1.0, rx=0.0, **walk)
    for _ in range(round(FLOOR_S / FIRMWARE_DT)):
        loop.tick()
    floor_z = sim.data.qpos[2]
    for _ in range(round((SECONDS - FLOOR_S) / FIRMWARE_DT)):
        loop.tick()
        past = sim.data.qpos[1] - y0 > face_at + 0.15
        if past and (sim.data.qpos[2] - floor_z) * 1000 > 0.7 * height_mm:
            return True
    return False


def climbs(height_mm: float, directory, **walk) -> int:
    """From how many of START_DISTANCES the walk gets onto the step."""
    return sum(walked_onto(height_mm, float(face_at), directory, **walk) for face_at in START_DISTANCES)


def test_the_climb_walks_onto_a_70_mm_step_from_every_start_the_tallest_classic_walk_mostly_cannot(tmp_path):
    assert climbs(70.0, tmp_path, climb=True, s=1.0) == len(START_DISTANCES)
    assert climbs(70.0, tmp_path, s1=1.0, h=-0.5, s=1.0) <= len(START_DISTANCES) // 2
