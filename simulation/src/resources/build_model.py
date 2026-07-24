"""Generate `model.xml` (MuJoCo MJCF) for the hexapod from the firmware geometry.

Generating the model (rather than hand-authoring 6 legs) keeps it consistent with
`firmware_gait.py` / `firmware/include/kinematics.h`. Units are SI (firmware mm / 1000).

Mass budget (total ~0.68 kg, measured on the real robot):
  - base/body (chassis + battery + ESP32): ~0.35 kg
  - per leg: coxa 0.010 + femur 0.030 + tibia 0.015 = 0.055 kg  -> 6 * 0.055 = 0.33 kg

MG92B servo model (@6V): ~0.08 s/60deg -> w_max ~ 13 rad/s; stall ~3.4 kg.cm ~ 0.33 N.m.

Run:  python src/resources/build_model.py             ->  writes src/resources/model.xml (flat plane)
      python src/resources/build_model.py --terrain   ->  also writes model_terrain.xml (heightfield
                                                          ground, filled per episode by src/sim/terrain.py)
"""

import math
import os
import sys

# --- geometry (mm), from firmware/include/kinematics.h ---
MOUNT_X = [44.82, 61.03, 44.82, -44.82, -61.03, -44.82]
MOUNT_Y = [74.82, 0.0, -74.82, 74.82, 0.0, -74.82]
MOUNT_ANGLE_DEG = [45.0, 0.0, -45.0, -225.0, -180.0, -135.0]
J1_J2 = 38.0
J2_J3 = 54.06
J3_TIP = 97.0

MM = 1e-3  # mm -> m

# --- masses (kg) ---  measured: total 0.68, each LEG 0.077.
# The hip/coxa servo is body-mounted (does NOT move with the leg) so its mass is in the base and the
# coxa link is a light bracket; the two lower-leg servos (femur+tibia) move with the leg.
M_BASE = 0.218   # whole body incl. the 6 body-mounted hip servos (split out as M_HIP below for inertia)
M_HIP = 0.016    # each body-mounted coxa/hip servo, placed at its leg MOUNT so base rotational inertia
                 # is realistic -- a uniform box under-estimates it badly (see build())
M_COXA = 0.010   # light bracket only (hip servo mass is in the base)
M_FEMUR = 0.042  # femur link + its servo
M_TIBIA = 0.025  # tibia link + servo; foot tip passive

# --- actuator (MG92B) ---
# Motor (torque) actuators; the servo PD + torque-speed clamp lives in mj_runtime (incl. position gain
# SERVO_KP), so the values here are only backstops for the XML.
FORCE = 0.50          # N.m torque envelope (see mj_runtime SERVO_STALL; real limit is the clamp)
CTRL_LIMIT = 0.9      # N.m motor ctrlrange -- loose backstop; real per-step limit is the torque-speed
                      # clamp in mj_runtime._apply_servo, not this cap
NOLOAD_SPEED = 13.0   # rad/s, no-load speed (~0.08 s/60deg for MG92B @6V)
ARMATURE = 0.003      # kg.m^2, reflected rotor+gearbox inertia (estimate; DR-scaled)
DAMPING = 0.02
FRICTIONLOSS = 0.005
COXA_RANGE = 0.55     # rad
FEMUR_RANGE = 1.8     # rad
TIBIA_RANGE = 2.2     # rad

# --- init/contact ---
INIT_Z = 0.075        # base spawn height (m); feet ~ -0.066 -> small clearance
FOOT_RADIUS = 0.006   # m
# Silicone tape on the real foot bottoms -> grippy. Foot geoms have priority=1 so this tangential mu
# wins over the ground's in every foot<->ground contact. High mu (no slip) but condim=3 (no torsional
# friction) keeps it grippy, not sticky: the foot won't slide but can still pivot freely.
FOOT_FRICTION = "2.0 0.02 0.001"

# --- terrain heightfield (model_terrain.xml) ---
HF_RADIUS = 5.0       # m, half-extent (matches the plane's rendered size)
HF_ZMAX = 0.15        # m, max elevation at hfield data = 1.0 (headroom; randomizer scales below this)
HF_N = 257            # grid rows/cols -> ~39 mm cells


def box_inertia(mass, lx, ly, lz):
    ix = mass * (ly * ly + lz * lz) / 12.0
    iy = mass * (lx * lx + lz * lz) / 12.0
    iz = mass * (lx * lx + ly * ly) / 12.0
    return ix, iy, iz


def leg_xml(i):
    mx, my = MOUNT_X[i] * MM, MOUNT_Y[i] * MM
    a = math.radians(MOUNT_ANGLE_DEG[i])
    l1, l2, l3 = J1_J2 * MM, J2_J3 * MM, J3_TIP * MM

    ci = box_inertia(M_COXA, l1, 0.02, 0.02)
    fi = box_inertia(M_FEMUR, l2, 0.015, 0.015)
    ti = box_inertia(M_TIBIA, l3, 0.012, 0.012)

    return f"""
      <body name="coxa_{i}" pos="{mx:.5f} {my:.5f} 0" euler="0 0 {a:.6f}">
        <joint name="coxa_{i}" type="hinge" axis="0 0 1" range="{-COXA_RANGE} {COXA_RANGE}"
               damping="{DAMPING}" frictionloss="{FRICTIONLOSS}" armature="{ARMATURE}"/>
        <inertial pos="{l1/2:.5f} 0 0" mass="{M_COXA}" diaginertia="{ci[0]:.3e} {ci[1]:.3e} {ci[2]:.3e}"/>
        <geom type="capsule" fromto="0 0 0 {l1:.5f} 0 0" size="0.008" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0"/>
        <body name="femur_{i}" pos="{l1:.5f} 0 0">
          <joint name="femur_{i}" type="hinge" axis="0 -1 0" range="{-FEMUR_RANGE} {FEMUR_RANGE}"
                 damping="{DAMPING}" frictionloss="{FRICTIONLOSS}" armature="{ARMATURE}"/>
          <inertial pos="{l2/2:.5f} 0 0" mass="{M_FEMUR}" diaginertia="{fi[0]:.3e} {fi[1]:.3e} {fi[2]:.3e}"/>
          <geom type="capsule" fromto="0 0 0 {l2:.5f} 0 0" size="0.007" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0"/>
          <body name="tibia_{i}" pos="{l2:.5f} 0 0">
            <joint name="tibia_{i}" type="hinge" axis="0 -1 0" range="{-TIBIA_RANGE} {TIBIA_RANGE}"
                   damping="{DAMPING}" frictionloss="{FRICTIONLOSS}" armature="{ARMATURE}"/>
            <inertial pos="{l3/2:.5f} 0 0" mass="{M_TIBIA}" diaginertia="{ti[0]:.3e} {ti[1]:.3e} {ti[2]:.3e}"/>
            <geom type="capsule" fromto="0 0 0 {l3:.5f} 0 0" size="0.005" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0"/>
            <geom name="foot_{i}" type="sphere" pos="{l3:.5f} 0 0" size="{FOOT_RADIUS}"
                  rgba="0.2 0.7 0.8 1" condim="3" friction="{FOOT_FRICTION}" priority="1"/>
            <site name="foot_{i}" pos="{l3:.5f} 0 0" size="0.004"/>
          </body>
        </body>
      </body>"""


def actuators_xml():
    rows = []
    for i in range(6):
        for joint, rng in (
            (f"coxa_{i}", COXA_RANGE),
            (f"femur_{i}", FEMUR_RANGE),
            (f"tibia_{i}", TIBIA_RANGE),
        ):
            rows.append(
                f'    <motor name="{joint}" joint="{joint}" '
                f'ctrlrange="{-CTRL_LIMIT} {CTRL_LIMIT}" gear="1"/>'  # ctrl = torque (N.m); PD+clamp in mj_runtime
            )
    return "\n".join(rows)


def build(terrain=False):
    # Base inertia: model the 6 hip servos as point masses at the leg MOUNTS. A uniform box of the whole
    # M_BASE under-estimates rotational inertia -> the body spins up too easily under leg-swing reaction
    # torques. Core (battery/PCB/frame) stays a centered box.
    ixx, iyy, izz = box_inertia(M_BASE - 6 * M_HIP, 0.070, 0.126, 0.038)
    for i in range(6):
        x, y = MOUNT_X[i] * MM, MOUNT_Y[i] * MM
        ixx += M_HIP * (y * y)          # parallel axis for a point mass at (x, y, 0)
        iyy += M_HIP * (x * x)
        izz += M_HIP * (x * x + y * y)
    bi = (ixx, iyy, izz)
    legs = "".join(leg_xml(i) for i in range(6))
    base_box_x, base_box_y, base_box_z = 0.035, 0.063, 0.019
    floor = """
    <texture name="grid" type="2d" builtin="checker" rgb1="0.18 0.26 0.42" rgb2="0.10 0.15 0.28"
             width="512" height="512"/>
    <material name="grid" texture="grid" texuniform="true" texrepeat="14 14" reflectance="0.35"/>"""
    if terrain:
        asset = f"""
  <asset>{floor}
    <hfield name="terrain" nrow="{HF_N}" ncol="{HF_N}" size="{HF_RADIUS} {HF_RADIUS} {HF_ZMAX} 0.1"/>
  </asset>
"""
        ground = """<geom name="ground" type="hfield" hfield="terrain" material="grid"
          condim="3" friction="1.0 0.02 0.001"/>"""
    else:
        asset = f"""
  <asset>{floor}
  </asset>
"""
        ground = """<geom name="ground" type="plane" material="grid" size="5 5 0.1"
          condim="3" friction="1.0 0.02 0.001"/>"""
    xml = f"""<mujoco model="hexapod">
  <compiler angle="radian" meshdir="stl" autolimits="true"/>
  <option timestep="0.002" iterations="10" solver="Newton" cone="elliptic" integrator="implicitfast">
    <flag eulerdamp="disable"/>
  </option>

  <default>
    <geom solref="0.01 1" solimp="0.9 0.95 0.001"/>
  </default>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/>
    <global azimuth="120" elevation="-20"/>
  </visual>
{asset}
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    {ground}

    <body name="base" pos="0 0 {INIT_Z}">
      <freejoint name="root"/>
      <inertial pos="0 0 0" mass="{M_BASE}" diaginertia="{bi[0]:.3e} {bi[1]:.3e} {bi[2]:.3e}"/>
      <geom name="chassis" type="box" size="{base_box_x} {base_box_y} {base_box_z}" rgba="0.9 0.9 0.9 1"
            contype="0" conaffinity="0"/>
      <site name="imu" pos="0 0 0" size="0.005"/>{legs}
    </body>
  </worldbody>

  <actuator>
{actuators_xml()}
  </actuator>

  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
  </sensor>
</mujoco>
"""
    name = "model_terrain.xml" if terrain else "model.xml"
    out = os.path.join(os.path.dirname(__file__), name)
    with open(out, "w", newline="\n") as f:
        f.write(xml)
    print(f"wrote {out}")


if __name__ == "__main__":
    build()
    if "--terrain" in sys.argv:
        build(terrain=True)
