// The virtual robot's C interface: the WebAssembly build the app's simulation view loads, and the native library
// the Python simulation loads (simulation/src/sim/virtual_robot.py). One robot per module instance or process;
// reset() starts it over.

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

}
