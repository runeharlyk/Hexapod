// The gesture path, from the PAJ7620U2's registers to the robot's walk: the driver against the register fake in
// test/fakes, the filter that keeps a hand reaching down from walking the robot, the move each gesture asks for in the
// body frame of docs/conventions.md, and the moves themselves walked by the virtual robot (firmware/host).

#define USE_POLICY 1
#define USE_MPU6050 1

#include <unity.h>

#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>
#include <gesture.h>
#include <imu_frame.h>
#include <peripherals/drivers/paj7620.h>
#include <virtual_robot.h>

namespace {

using gesture::Hand;

constexpr int TICKS_PER_SECOND = 200;
constexpr float GOAL_LIMIT_S = 40;
constexpr unsigned long POLL_MS = 50;

int ticksFor(float seconds) { return static_cast<int>(std::lround(seconds * TICKS_PER_SECOND)); }

bool running(GoalState s) { return s == GoalState::SETTLING || s == GoalState::RUNNING; }

// The pose `end` reached from `start`, in the frame the robot faced at `start`.
OdometryEstimate relative(const OdometryEstimate &start, const OdometryEstimate &end) {
    const float dx = end.x - start.x, dy = end.y - start.y, c = std::cos(start.yaw), s = std::sin(start.yaw);
    return {.x = c * dx + s * dy, .y = -s * dx + c * dy, .yaw = std::remainder(end.yaw - start.yaw, 2 * PI_F)};
}

// The robot on the floor, its feet not slipping, so its DMP's yaw is the turn its feet made. It takes a gesture as
// main.cpp's gesture task hands one to the Hexapod.
struct Rig {
    VirtualRobot robot;

    void tick() {
        Peripherals::Imu imu;
        imu.rpyRad[2] = imu_frame::YAW_SIGN * robot.odometry().kinematicYaw;
        robot.setImu(imu);
        robot.tick();
    }

    void run(float seconds) {
        for (int t = 0; t < ticksFor(seconds); ++t) tick();
    }

    GoalState state() const { return static_cast<GoalState>(robot.goalStatus().state); }

    // Ticks until the goal ends, `during` called after each tick.
    void runGoal(const std::function<void()> &during = {}) {
        for (int t = 0; t < ticksFor(GOAL_LIMIT_S) && running(state()); ++t) {
            tick();
            if (during) during();
        }
    }

    bool still() const { return robot.still(); }
    int startGoal(const Goal &goal, const CommandMsg &style, GaitType gait, GoalOwner owner) {
        return robot.goalMove(goal, style, gait, owner);
    }
};

// One rig at a time, as one VirtualRobot exists at a time; deleted by tearDown when a failed assertion leaves a test.
std::unique_ptr<Rig> currentRig;

Rig &standingRig() {
    currentRig.reset();
    currentRig = std::make_unique<Rig>();
    currentRig->robot.setMode(MOTION_STATE::STAND);
    currentRig->run(1.0f);
    return *currentRig;
}

// What the filter confirms of `seen` at each poll from 0 to `untilMs`, with the poll at which it did.
struct Confirmed {
    Hand hand;
    unsigned long atMs;
};
std::vector<Confirmed> filtered(const std::vector<std::pair<unsigned long, Hand>> &seen, unsigned long untilMs) {
    gesture::Filter filter;
    std::vector<Confirmed> out;
    for (unsigned long t = 0; t <= untilMs; t += POLL_MS) {
        Hand hand = Hand::NONE;
        for (const auto &[atMs, h] : seen)
            if (atMs == t) hand = h;
        if (const std::optional<Hand> confirmed = filter.update(hand, t)) out.push_back({*confirmed, t});
    }
    return out;
}

void giveChipItsId() {
    I2CBus::instance().registers[PAJ7620Driver::ADDR][0x00] = 0x20;
    I2CBus::instance().registers[PAJ7620Driver::ADDR][0x01] = 0x76;
}

} // namespace

void setUp() {
    host_bus::resetAll();
    I2CBus::instance().reset();
}
void tearDown() {
    currentRig.reset();
    Peripherals::setImu({});
}

// No chip, or one with another part ID, is no sensor: nothing past the two wake-up writes, so the robot runs on
// without gestures.
void test_begin_refuses_a_missing_chip_and_one_with_another_id() {
    PAJ7620Driver sensor;
    TEST_ASSERT_FALSE(sensor.begin());
    I2CBus::instance().registers[PAJ7620Driver::ADDR][0x00] = 0x20;
    TEST_ASSERT_FALSE(sensor.begin());
    TEST_ASSERT_EQUAL(2, I2CBus::instance().writes.size());
}

// The chip takes the vendor's whole initialisation, in order, and is left on bank 0, where the flags are read.
void test_begin_initialises_the_chip_and_leaves_it_on_bank_zero() {
    giveChipItsId();
    PAJ7620Driver sensor;
    TEST_ASSERT_TRUE(sensor.begin());
    const std::vector<FakeWrite> &writes = I2CBus::instance().writes;
    TEST_ASSERT_EQUAL(2 + 219 + 1, writes.size());
    TEST_ASSERT_EQUAL_HEX8(0x32, writes[3].reg);
    TEST_ASSERT_EQUAL_HEX8(0x29, writes[3].bytes[0]);
    TEST_ASSERT_EQUAL_HEX8(0x7E, writes[2 + 218].reg);
    TEST_ASSERT_EQUAL_HEX8(0xEF, writes.back().reg);
    TEST_ASSERT_EQUAL_HEX8(0x00, writes.back().bytes[0]);
}

// An init write the chip refuses fails the start, as a sensor half set up reads nothing it can be trusted for.
void test_begin_fails_when_an_init_write_is_refused() {
    giveChipItsId();
    I2CBus::instance().writeOverride = [](uint8_t, uint8_t reg, const uint8_t *, size_t) -> std::optional<esp_err_t> {
        return reg == 0x90 ? std::optional<esp_err_t>(ESP_FAIL) : std::nullopt;
    };
    PAJ7620Driver sensor;
    TEST_ASSERT_FALSE(sensor.begin());
}

// The wave's register over the sweeps', so one read tells a wave from a sweep; a failed read is no reading at all,
// not an empty one.
void test_the_flags_read_put_the_wave_over_the_sweeps_and_a_failed_read_reads_nothing() {
    giveChipItsId();
    I2CBus::instance().registers[PAJ7620Driver::ADDR][0x43] = 0x04;
    I2CBus::instance().registers[PAJ7620Driver::ADDR][0x44] = 0x01;
    PAJ7620Driver sensor;
    TEST_ASSERT_EQUAL_HEX16(0x0104, sensor.readFlags().value_or(0));
    I2CBus::instance().readOverride = [](uint8_t, uint8_t reg, uint8_t *, size_t) -> std::optional<esp_err_t> {
        return reg == 0x43 ? std::optional<esp_err_t>(ESP_FAIL) : std::nullopt;
    };
    TEST_ASSERT_FALSE(sensor.readFlags().has_value());
}

// A wave over anything else, then the vendor's bit order; no flag is no gesture.
void test_decode_takes_the_wave_first_then_the_vendors_order() {
    TEST_ASSERT_EQUAL(Hand::WAVE, gesture::decode(0x0101));
    TEST_ASSERT_EQUAL(Hand::RIGHT, gesture::decode(0x0001));
    TEST_ASSERT_EQUAL(Hand::LEFT, gesture::decode(0x0002));
    TEST_ASSERT_EQUAL(Hand::UP, gesture::decode(0x0004));
    TEST_ASSERT_EQUAL(Hand::DOWN, gesture::decode(0x0008));
    TEST_ASSERT_EQUAL(Hand::TOWARD, gesture::decode(0x0010));
    TEST_ASSERT_EQUAL(Hand::AWAY, gesture::decode(0x0020));
    TEST_ASSERT_EQUAL(Hand::CLOCKWISE, gesture::decode(0x0040));
    TEST_ASSERT_EQUAL(Hand::COUNTER_CLOCKWISE, gesture::decode(0x0080));
    TEST_ASSERT_EQUAL(Hand::RIGHT, gesture::decode(0x0011));
    TEST_ASSERT_EQUAL(Hand::NONE, gesture::decode(0x0000));
}

// A sweep is acted on once, at the first poll CONFIRM_MS after it, and a second one meanwhile replaces nothing.
void test_a_sweep_is_confirmed_once_after_the_entry_delay() {
    const std::vector<Confirmed> out = filtered({{100, Hand::UP}, {400, Hand::LEFT}}, 3000);
    TEST_ASSERT_EQUAL(1, out.size());
    TEST_ASSERT_EQUAL(Hand::UP, out[0].hand);
    TEST_ASSERT_EQUAL(100 + gesture::Filter::CONFIRM_MS, out[0].atMs);
}

// A hand that crosses the chip and then comes toward it, as one reaching down to lift the robot, walks nothing; nor
// does anything seen while it leaves, the quit delay. A sweep after that is taken again.
void test_a_hand_coming_down_cancels_the_sweep_before_it_and_quiets_the_sensor() {
    TEST_ASSERT_EQUAL(0, filtered({{0, Hand::RIGHT}, {500, Hand::TOWARD}}, 3000).size());
    TEST_ASSERT_EQUAL(0, filtered({{0, Hand::WAVE}, {500, Hand::UP}}, 3000).size());
    const std::vector<Confirmed> out = filtered({{0, Hand::AWAY}, {500, Hand::UP}, {1000, Hand::DOWN}}, 3000);
    TEST_ASSERT_EQUAL(1, out.size());
    TEST_ASSERT_EQUAL(Hand::DOWN, out[0].hand);
    TEST_ASSERT_EQUAL(1000 + gesture::Filter::CONFIRM_MS, out[0].atMs);
}

// Sweeps walk 40 cm the way the hand went, forward along +y and right along +x, as the sensor is mounted; circles
// turn half a turn their way, counter-clockwise positive; the hand coming or going and the wave move nothing.
void test_each_gesture_asks_for_its_move_in_the_body_frame() {
    struct Case {
        Hand hand;
        int quarters;
        GoalKind kind;
        float distance, angle;
    };
    const Case cases[] = {
        {Hand::UP, 0, GoalKind::FORWARD, 0.4f, 0},
        {Hand::DOWN, 0, GoalKind::FORWARD, -0.4f, 0},
        {Hand::RIGHT, 0, GoalKind::SIDEWAYS, 0.4f, 0},
        {Hand::LEFT, 0, GoalKind::SIDEWAYS, -0.4f, 0},
        // The chip's UP pointing to the robot's right: a sweep toward its LEFT edge walks forward.
        {Hand::UP, 1, GoalKind::SIDEWAYS, 0.4f, 0},
        {Hand::LEFT, 1, GoalKind::FORWARD, 0.4f, 0},
        {Hand::UP, -1, GoalKind::SIDEWAYS, -0.4f, 0},
        {Hand::CLOCKWISE, 1, GoalKind::TURN, 0, -PI_F},
        {Hand::COUNTER_CLOCKWISE, 0, GoalKind::TURN, 0, PI_F},
    };
    for (const Case &c : cases) {
        const std::optional<Goal> goal = gesture::goalFor(c.hand, c.quarters);
        TEST_ASSERT_TRUE(goal.has_value());
        TEST_ASSERT_EQUAL(c.kind, goal->kind);
        TEST_ASSERT_EQUAL_FLOAT(c.distance, goal->distance);
        TEST_ASSERT_EQUAL_FLOAT(c.angle, goal->angle);
        TEST_ASSERT_TRUE(GoalController::valid(*goal));
    }
    for (const Hand hand : {Hand::NONE, Hand::TOWARD, Hand::AWAY, Hand::WAVE})
        TEST_ASSERT_FALSE(gesture::goalFor(hand, 0).has_value());
}

// From a stand, a sweep walks the robot 40 cm forward and leaves it in WALK at rest; a sweep from there is taken too
// and walks it 40 cm to its right.
void test_sweeps_walk_forty_centimetres_one_after_another() {
    Rig &rig = standingRig();
    TEST_ASSERT_TRUE(rig.still());
    OdometryEstimate start = rig.robot.odometry();
    TEST_ASSERT_EQUAL_INT(202, gesture::act(Hand::UP, rig));
    rig.runGoal();
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
    OdometryEstimate walked = relative(start, rig.robot.odometry());
    TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_M, 0.4f, walked.y);
    TEST_ASSERT_FLOAT_WITHIN(GoalController::CROSS_REACHED_M, 0, walked.x);

    rig.run(FEET_EASE_S);
    TEST_ASSERT_EQUAL(MOTION_STATE::WALK, rig.robot.mode());
    TEST_ASSERT_TRUE(rig.still());
    start = rig.robot.odometry();
    TEST_ASSERT_EQUAL_INT(202, gesture::act(Hand::RIGHT, rig));
    rig.runGoal();
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
    walked = relative(start, rig.robot.odometry());
    TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_M, 0.4f, walked.x);
    TEST_ASSERT_FLOAT_WITHIN(GoalController::CROSS_REACHED_M, 0, walked.y);
}

// A circle turns the robot around the way the hand went: halfway it has turned that way, and it ends half a turn on.
void test_circles_turn_the_robot_around_their_way() {
    for (const auto &[hand, sign] : {std::pair {Hand::CLOCKWISE, -1.f}, std::pair {Hand::COUNTER_CLOCKWISE, 1.f}}) {
        Rig &rig = standingRig();
        const OdometryEstimate start = rig.robot.odometry();
        TEST_ASSERT_EQUAL_INT(202, gesture::act(hand, rig));
        float halfway = 0;
        rig.runGoal([&] {
            if (halfway == 0 && rig.robot.goalStatus().progress >= 0.5f)
                halfway = relative(start, rig.robot.odometry()).yaw;
        });
        TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
        TEST_ASSERT_FLOAT_WITHIN(PI_F / 4, sign * PI_F / 2, halfway);
        TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_RAD, PI_F,
                                 std::fabs(relative(start, rig.robot.odometry()).yaw));
    }
}

// Only a robot standing still takes a gesture: not one deactivated, not one walking a goal (whose goal goes on), not
// one accepted a moment ago and not yet walked, and not one a client steers.
void test_a_gesture_is_refused_unless_the_robot_stands_still() {
    currentRig.reset();
    currentRig = std::make_unique<Rig>();
    Rig &off = *currentRig;
    off.run(0.5f);
    TEST_ASSERT_EQUAL_INT(409, gesture::act(Hand::UP, off));
    TEST_ASSERT_EQUAL_UINT32(0, off.robot.goalStatus().id);

    Rig &rig = standingRig();
    TEST_ASSERT_EQUAL_INT(202, gesture::act(Hand::UP, rig));
    TEST_ASSERT_EQUAL_INT(409, gesture::act(Hand::DOWN, rig));
    rig.run(1.5f);
    TEST_ASSERT_EQUAL(GoalState::RUNNING, rig.state());
    const uint32_t id = rig.robot.goalStatus().id;
    TEST_ASSERT_EQUAL_INT(409, gesture::act(Hand::LEFT, rig));
    rig.run(0.5f);
    TEST_ASSERT_EQUAL_UINT32(id, rig.robot.goalStatus().id);
    TEST_ASSERT_EQUAL(GoalState::RUNNING, rig.state());

    Rig &steered = standingRig();
    steered.robot.setMode(MOTION_STATE::WALK);
    for (int t = 0; t < ticksFor(1.0f); ++t) {
        if (t % 4 == 0) steered.robot.command({0, 0.5f, 0, 0, 0, 0, 0, 0, 0});
        steered.tick();
    }
    TEST_ASSERT_EQUAL_INT(409, gesture::act(Hand::UP, steered));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_begin_refuses_a_missing_chip_and_one_with_another_id);
    RUN_TEST(test_begin_initialises_the_chip_and_leaves_it_on_bank_zero);
    RUN_TEST(test_begin_fails_when_an_init_write_is_refused);
    RUN_TEST(test_the_flags_read_put_the_wave_over_the_sweeps_and_a_failed_read_reads_nothing);
    RUN_TEST(test_decode_takes_the_wave_first_then_the_vendors_order);
    RUN_TEST(test_a_sweep_is_confirmed_once_after_the_entry_delay);
    RUN_TEST(test_a_hand_coming_down_cancels_the_sweep_before_it_and_quiets_the_sensor);
    RUN_TEST(test_each_gesture_asks_for_its_move_in_the_body_frame);
    RUN_TEST(test_sweeps_walk_forty_centimetres_one_after_another);
    RUN_TEST(test_circles_turn_the_robot_around_their_way);
    RUN_TEST(test_a_gesture_is_refused_unless_the_robot_stands_still);
    return UNITY_END();
}
