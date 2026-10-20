// The virtual robot's C interface: the WebAssembly build the app's simulation view loads, and the native library
// the Python simulation loads (simulation/src/sim/virtual_robot.py). One robot per module instance or process;
// reset() starts it over.

#include <algorithm>
#include <iterator>
#include <gait_speed.h>
#include <host_api.h>
#include <motion_state_wire.h>
#include <pb_encode.h>
#include <virtual_robot.h>

namespace {

// Never destroyed at exit: its destructor drains the EventBus, whose static queues may already be gone by then, so
// a library unloaded at process exit would crash. reset() replaces it while they are all alive.
VirtualRobot *robot = nullptr;

VirtualRobot &instance() {
    if (!robot) robot = new VirtualRobot();
    return *robot;
}

}  // namespace

extern "C" {

// The clock starts over too, so a new robot's run does not depend on the runs before it in the process.
API void reset() {
    delete robot;
    robot = nullptr;
    host_clock::nowUs() = 1;
    instance();
}

// A mode by its ModesEnum value; one the robot has no mode for, POSE's reserved 2 among them, is ignored.
API void setMode(int mode) {
    if (const std::optional<MOTION_STATE> m = motionStateFromWire(mode)) instance().setMode(*m);
}

API void setGait(int gait) { instance().setGait(static_cast<GaitType>(gait)); }

// A ControllerInputData message, as a client sends it.
API void command(float lx, float ly, float rx, float ry, float h, float s, float s1, float fd, float stride) {
    socket_message_ControllerInputData in = socket_message_ControllerInputData_init_zero;
    in.has_left = in.has_right = true;
    in.left = {lx, ly};
    in.right = {rx, ry};
    in.height = h;
    in.speed = s;
    in.s1 = s1;
    in.feet_distance = fd;
    in.stride = stride;
    instance().input(in);
}

// Roll, pitch and yaw in radians as the MPU6050 driver reports them, the body-frame gravity unit vector
// pointing up, and the body-frame rates in rad/s.
API void setImu(float roll, float pitch, float yaw, float gx, float gy, float gz, float wx, float wy,
                                 float wz) {
    instance().setImu({{roll, pitch, yaw}, {gx, gy, gz}, {wx, wy, wz}});
}

// One 5 ms control tick.
API void tick() { instance().tick(); }

// A GoalMove request, as main.cpp's handler takes it, from a client with no transport, so no silence ends it: the
// reply's status, 202 when the goal starts on the next tick.
API int goalMove(int kind, float distance, float angle, float radius, float speed, int gait, float stride, float s1,
                 float h, float fd) {
    if (kind < 0 || kind > static_cast<int>(GoalKind::ARC)) return 400;
    const Goal goal {static_cast<GoalKind>(kind), distance, angle, radius, speed};
    const CommandMsg style {0, 0, 0, 0, h, 0, s1, fd, stride};
    return instance().goalMove(goal, style, static_cast<GaitType>(gait));
}

API void goalCancel() { instance().goalCancel(); }

// The newest goal's GoalStatus, its fields in their protobuf order: id, state, kind, target, progress, x, y, yaw and
// passes.
API void goalStatus(float out[9]) {
    const socket_message_GoalStatus s = instance().goalStatus();
    const float fields[9] = {static_cast<float>(s.id),
                             static_cast<float>(s.state),
                             static_cast<float>(s.kind),
                             s.target,
                             s.progress,
                             s.x,
                             s.y,
                             s.yaw,
                             static_cast<float>(s.passes)};
    std::copy(std::begin(fields), std::end(fields), out);
}

// The pose after the last tick, as OdometryData's fields in their order: x, y, yaw, kinematic yaw and distance.
API void odometry(float out[5]) {
    const OdometryEstimate e = instance().odometry();
    const float fields[5] = {e.x, e.y, e.yaw, e.kinematicYaw, e.distance};
    std::copy(std::begin(fields), std::end(fields), out);
}

// Takes effect on the next tick; false, and nothing reset, while a goal drives.
API bool resetOdometry() { return instance().resetOdometry(); }

// Whether the servos are driven, and the 18 joint angles they are driven to in degrees, three per leg (coxa, femur,
// tibia) in leg order.
API bool servosDriven() { return instance().servosDriven(); }

API const float *servoAngles() { return instance().servoAngles(); }

// Poses sent to the servos since reset().
API unsigned servoWrites() { return instance().servoWrites(); }

// A JointAngleData message, as main.cpp's handler passes it on: one joint in degrees, taken in IDLE alone.
API void setJoint(int servo, float degrees) { instance().setJoint(servo, degrees); }

// Single joints written in IDLE since reset().
API unsigned jointWrites() { return instance().jointWrites(); }

// The last tick's TickSample encoded as the robot's telemetry carries it, without the control loop's own fields (seq,
// t_us, period_us, compute_us): the bytes written into `out`, or 0 when they do not fit in `size`.
API int telemetrySample(uint8_t *out, int size) {
    pb_ostream_t stream = pb_ostream_from_buffer(out, static_cast<size_t>(size));
    return pb_encode(&stream, socket_message_TickSample_fields, &instance().sample())
               ? static_cast<int>(stream.bytes_written)
               : 0;
}

// gait_speed.h on its own, no robot needed, for the simulation's port to be checked against: a classic gait's walk
// command for a stick at a posture and stride, as step x, step y, step angle and phase rate.
API void classicWalkTarget(int gait, float lx, float ly, float rx, float s, float s1, float h, float fd, float stride,
                           float out[4]) {
    const WalkTarget w = ::classicWalkTarget(static_cast<GaitType>(gait), lx, ly, rx, s, s1, h, fd, stride);
    out[0] = w.stepX;
    out[1] = w.stepY;
    out[2] = w.stepAngle;
    out[3] = w.phaseRate;
}

// AUTO's gait for a stick at a posture and stride, chosen from the gait in effect `current`, and the walk it takes
// there as classicWalkTarget's four values.
API int autoWalk(int current, float lx, float ly, float rx, float s, float s1, float h, float fd, float stride,
                 float out[4]) {
    const AutoDemand demand = autoDemand(lx, ly, rx, s, s1, h, fd, stride);
    const GaitType gait = selectAutoGait(demand, static_cast<GaitType>(current), s1, h, fd, stride);
    const WalkTarget w = autoWalkTarget(gait, demand, lx, ly, rx, s1, h, fd, stride);
    out[0] = w.stepX;
    out[1] = w.stepY;
    out[2] = w.stepAngle;
    out[3] = w.phaseRate;
    return static_cast<int>(gait);
}

// The posture s1, h, fd pulled into the walkable region, in place.
API void clampPosture(float posture[3]) { ::clampPosture(posture[0], posture[1], posture[2]); }
}
