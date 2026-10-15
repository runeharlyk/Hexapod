// The virtual robot's C interface for the WebAssembly build that the app's simulation view loads. One robot
// per module instance; reset() starts it over.

#include <memory>
#include <emscripten/emscripten.h>
#include <virtual_robot.h>

namespace {

std::unique_ptr<VirtualRobot> robot;

VirtualRobot &instance() {
    if (!robot) robot = std::make_unique<VirtualRobot>();
    return *robot;
}

}  // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE void reset() {
    robot.reset();
    instance();
}

EMSCRIPTEN_KEEPALIVE void setMode(int mode) { instance().setMode(static_cast<MOTION_STATE>(mode)); }

EMSCRIPTEN_KEEPALIVE void setGait(int gait) { instance().setGait(static_cast<GaitType>(gait)); }

EMSCRIPTEN_KEEPALIVE void command(float lx, float ly, float rx, float ry, float h, float s, float s1, float fd) {
    instance().command({lx, ly, rx, ry, h, s, s1, fd});
}

// Roll, pitch and yaw in degrees as the MPU6050 DMP reports them, the body-frame gravity unit vector
// pointing up, and the body-frame rates in rad/s.
EMSCRIPTEN_KEEPALIVE void setImu(float roll, float pitch, float yaw, float gx, float gy, float gz, float wx, float wy,
                                 float wz) {
    instance().setImu({{roll, pitch, yaw}, {gx, gy, gz}, {wx, wy, wz}});
}

// One 5 ms control tick.
EMSCRIPTEN_KEEPALIVE void tick() { instance().tick(); }

// The last tick's 18 joint angles in degrees, three per leg (coxa, femur, tibia) in leg order.
EMSCRIPTEN_KEEPALIVE const float *angles() { return instance().angles(); }

EMSCRIPTEN_KEEPALIVE int mode() { return static_cast<int>(instance().mode()); }

EMSCRIPTEN_KEEPALIVE int gait() { return static_cast<int>(instance().gait()); }

}
