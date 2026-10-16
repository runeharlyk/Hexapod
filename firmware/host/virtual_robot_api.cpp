// The virtual robot's C interface: the WebAssembly build the app's simulation view loads, and the native library
// the Python simulation loads (simulation/src/sim/virtual_robot.py). One robot per module instance or process;
// reset() starts it over.

#include <gait_speed.h>
#include <host_api.h>
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

API void reset() {
    delete robot;
    robot = nullptr;
    instance();
}

API void setMode(int mode) { instance().setMode(static_cast<MOTION_STATE>(mode)); }

API void setGait(int gait) { instance().setGait(static_cast<GaitType>(gait)); }

API void command(float lx, float ly, float rx, float ry, float h, float s, float s1, float fd) {
    instance().command({lx, ly, rx, ry, h, s, s1, fd});
}

// Roll, pitch and yaw in radians as the MPU6050 driver reports them, the body-frame gravity unit vector
// pointing up, and the body-frame rates in rad/s.
API void setImu(float roll, float pitch, float yaw, float gx, float gy, float gz, float wx, float wy,
                                 float wz) {
    instance().setImu({{roll, pitch, yaw}, {gx, gy, gz}, {wx, wy, wz}});
}

// One 5 ms control tick.
API void tick() { instance().tick(); }

// Whether the servos are driven, and the 18 joint angles they are driven to in degrees, three per leg (coxa, femur,
// tibia) in leg order.
API bool servosDriven() { return instance().servosDriven(); }

API const float *servoAngles() { return instance().servoAngles(); }

// Poses sent to the servos since reset().
API unsigned servoWrites() { return instance().servoWrites(); }

// gait_speed.h on its own, no robot needed, for the simulation's port to be checked against: a classic gait's walk
// command for a stick at a posture, as step x, step y, step angle and phase rate.
API void classicWalkTarget(int gait, float lx, float ly, float rx, float s, float s1, float h, float fd, float out[4]) {
    const WalkTarget w = ::classicWalkTarget(static_cast<GaitType>(gait), lx, ly, rx, s, s1, h, fd, 0);
    out[0] = w.stepX;
    out[1] = w.stepY;
    out[2] = w.stepAngle;
    out[3] = w.phaseRate;
}

// AUTO's gait for a stick at a posture, chosen from the gait in effect `current`, and the walk it takes there as
// classicWalkTarget's four values.
API int autoWalk(int current, float lx, float ly, float rx, float s, float s1, float h, float fd, float out[4]) {
    const AutoDemand demand = autoDemand(lx, ly, rx, s, s1, h, fd, 0);
    const GaitType gait = selectAutoGait(demand, static_cast<GaitType>(current), s1, h, fd, 0);
    const WalkTarget w = autoWalkTarget(gait, demand, lx, ly, rx, s1, h, fd, 0);
    out[0] = w.stepX;
    out[1] = w.stepY;
    out[2] = w.stepAngle;
    out[3] = w.phaseRate;
    return static_cast<int>(gait);
}

// The posture s1, h, fd pulled into the walkable region, in place.
API void clampPosture(float posture[3]) { ::clampPosture(posture[0], posture[1], posture[2]); }
}
