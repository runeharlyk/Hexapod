import math
from typing import Callable, List, Tuple, TypeVar, Any, TypedDict
from kinematics import compute_default_position

class GaitStateT(TypedDict):
    step_height: float
    step_x: float
    step_z: float
    step_angle: float
    step_velocity: float
    step_depth: float

class BezierState:
    def __init__(self, num_legs=6, num_phases=2, cycle_period=1.0):
        self.name = 'Bezier'
        self.num_legs = num_legs
        self.num_phases = num_phases
        self.cycle_period = cycle_period
        self.phase = 0
        self.phase_num = 0
        self.step_length = 0
        self.offset = [0.5, 0, 0, 0.5, 0.5, 0]#[(0.0 if i % num_phases == 0 else cycle_period / num_phases) for i in range(num_legs)]
        self.default_feet_pos = compute_default_position()

    def step(self, body_state, gait_state, dt=0.02):
        self.body_state = body_state
        self.gait_state = gait_state
        self.dt = dt
        self.step_length = math.sqrt(self.gait_state["step_x"] ** 2 + self.gait_state["step_z"] ** 2)
        if self.gait_state["step_x"] < 0:
            self.step_length = -self.step_length
        self.update_phase()
        self.update_feet_positions()
        return self.body_state

    def update_phase(self):
        self.phase += self.dt * self.gait_state["step_velocity"] * 2
        if self.phase >= 1:
            self.phase_num += 1
            self.phase_num %= 2
            self.phase = 0

    def update_feet_positions(self):
        for i in range(self.num_legs):
            self.body_state["feet"][i] = self.update_foot_position(i)

    def update_foot_position(self, index):
        phase = self.phase + self.offset[index]
        if phase >= 1:
            phase -= 1
        self.body_state["feet"][index][0] = self.default_feet_pos[index][0]
        self.body_state["feet"][index][1] = self.default_feet_pos[index][1]
        self.body_state["feet"][index][2] = self.default_feet_pos[index][2]
        return self.stand_controller(index, phase / 0.60) if phase <= 0.60 else self.swing_controller(index, (phase - 0.60) / (1 - 0.60))

    def stand_controller(self, index, phase):
        depth = self.gait_state["step_depth"]
        return self.controller(index, phase, stance_curve, depth)

    def swing_controller(self, index, phase):
        height = self.gait_state["step_height"]
        return self.default_feet_pos[index] #self.controller(index, phase, bezier_curve, height)

    def controller(self, index, phase, controller_func, *args):
        length = self.step_length / 2
        angle = math.atan2(self.gait_state["step_z"], self.gait_state["step_x"]) if self.gait_state["step_x"] != 0 else math.pi/2
        delta_pos = controller_func(length, angle, *args, phase)

        length = self.gait_state["step_angle"] * 2
        angle = yaw_arc(self.default_feet_pos[index], self.body_state["feet"][index])

        delta_rot = controller_func(length, angle, *args, phase)

        self.body_state["feet"][index][0] += delta_pos[0] + delta_rot[0] * 0.2
        self.body_state["feet"][index][1] += delta_pos[1] + delta_rot[1] * 0.2
        self.body_state["feet"][index][2] += delta_pos[2] + delta_rot[2] * 0.2

        return self.body_state["feet"][index]

def stance_curve(length, angle, depth, phase):
    x_polar = math.cos(angle)
    y_polar = math.sin(angle)

    step = length * (1 - 2 * phase)
    x = step * x_polar
    y = step * y_polar
    z = 0

    if length != 0:
        z = -depth * math.cos((math.pi * (x + y)) / (2 * length))
    return [x, y, z]

def yaw_arc(default_foot_pos, current_foot_pos):
    foot_mag = math.sqrt(default_foot_pos[0] ** 2 + default_foot_pos[1] ** 2)
    foot_dir = math.atan2(default_foot_pos[1], default_foot_pos[0])
    offsets = [
        current_foot_pos[0] - default_foot_pos[0],
        current_foot_pos[1] - default_foot_pos[1],
        current_foot_pos[2] - default_foot_pos[2]
    ]
    offset_mag = math.sqrt(offsets[0] ** 2 + offsets[1] ** 2)
    offset_mod = math.atan2(offset_mag, foot_mag)

    return math.pi / 2.0 + foot_dir + offset_mod

def bezier_curve(length, angle, height, phase):
    control_points = get_control_points(length, angle, height)
    n = len(control_points) - 1

    point = [0, 0, 0]
    for i in range(n + 1):
        bernstein_poly = comb(n, i) * (phase ** i) * ((1 - phase) ** (n - i))
        point[0] += bernstein_poly * control_points[i][0]
        point[1] += bernstein_poly * control_points[i][1]
        point[2] += bernstein_poly * control_points[i][2]
    return point

def get_control_points(length, angle, height):
    x_polar = math.cos(angle)
    y_polar = math.sin(angle)

    step = [
        -length,
        -length * 1.4,
        -length * 1.5,
        -length * 1.5,
        -length * 1.5,
        0.0,
        0.0,
        0.0,
        length * 1.5,
        length * 1.5,
        length * 1.4,
        length
    ]

    z = [
        0.0,
        0.0,
        height * 0.9,
        height * 0.9,
        height * 0.9,
        height * 0.9,
        height * 0.9,
        height * 1.1,
        height * 1.1,
        height * 1.1,
        0.0,
        0.0
    ]

    control_points = []

    for i in range(len(step)):
        x = step[i] * x_polar
        y = step[i] * y_polar
        control_points.append([x, y, z[i]])

    return control_points

def comb(n, k):
    if k < 0 or k > n:
        return 0
    if k == 0 or k == n:
        return 1
    k = min(k, n - k)
    c = 1
    for i in range(k):
        c = (c * (n - i)) // (i + 1)
    return c

def gait_leg_phase(t, leg_index, T=1.0):
    phase = (t / T + leg_phase_offset(leg_index)) % 1.0
    return phase

def leg_phase_offset(i):
    return 0.0 if i in (0, 3, 4) else 0.5

def gait_position(local_default, t, leg_index, T=1.0, lift_height=3.0, step_size=3.0):
    phase = gait_leg_phase(t, leg_index, T)
    
    if phase < 0.5:
        alpha = phase / 0.5
        forward_offset = step_size * (alpha - 0.5)
        height_offset = lift_height * (1 - abs(2*alpha - 1))
    else:
        alpha = (phase - 0.5) / 0.5
        forward_offset = -step_size * 0.5
        height_offset = 0

    return (local_default[0] + forward_offset, local_default[1], local_default[2] + height_offset)

def compute_gait_positions(default_positions_local, t, T=1.0):
    positions = []
    for i in range(6):
        pos = gait_position(default_positions_local[i], t, i, T)
        positions.append(pos)
    return positions