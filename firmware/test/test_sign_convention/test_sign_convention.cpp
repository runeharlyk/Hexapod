// Host test of the body-frame sign convention against platform_shared/sign_convention.json, which the
// app's gait.ts and the simulation's firmware_gait.py pass too, so no copy of the gait can mirror an
// axis on its own. docs/conventions.md derives the expected signs.

#include <unity.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <gait.h>
#include <kinematics.h>

namespace {

struct Command {
    std::string name;
    float stepX, stepY, stepAngle;
    int bodyVelocitySign[2];
    int yawSign;
    int footSign[6][2];
};

struct Fixture {
    std::vector<std::string> legs;
    float stance[6][3];
    float dt;
    int ticks;
    float zeroMmPerTick, zeroRadPerTick;
    std::vector<Command> commands;
};

// Reads only the layout the fixture file has: keys in its order, no escapes in strings. A missing key
// or a short list clears `ok` instead of reading out of bounds.
struct Reader {
    const std::string& json;
    size_t at = 0;
    bool ok = true;

    size_t after(const std::string& key, char what) {
        const size_t keyAt = json.find(key, at);
        const size_t found = keyAt == std::string::npos ? keyAt : json.find(what, keyAt + key.size());
        if (found == std::string::npos) ok = false;
        return found;
    }

    bool has(const std::string& key) const { return json.find(key, at) != std::string::npos; }

    // The text inside the bracket pair that opens first after `key`.
    std::string bracketed(const std::string& key, char open, char close) {
        const size_t start = after(key, open);
        if (!ok) return "";
        int depth = 0;
        size_t end = start;
        do {
            if (json[end] == open) depth++;
            if (json[end] == close) depth--;
            end++;
        } while (depth > 0 && end < json.size());
        if (depth != 0) ok = false;
        at = end;
        return json.substr(start + 1, end - start - 2);
    }

    std::vector<float> numbers(const std::string& key, char open, char close, size_t count) {
        const std::string text = bracketed(key, open, close);
        std::vector<float> out;
        const char* p = text.c_str();
        while (*p) {
            char* next;
            const float value = std::strtof(p, &next);
            if (next == p) {
                p++;
            } else {
                out.push_back(value);
                p = next;
            }
        }
        if (out.size() != count) {
            ok = false;
            out.assign(count, 0.0f);
        }
        return out;
    }

    std::vector<std::string> strings(const std::string& key) {
        const std::string list = bracketed(key, '[', ']');
        std::vector<std::string> out;
        for (size_t open = list.find('"'); open != std::string::npos; open = list.find('"', open + 1)) {
            const size_t close = list.find('"', open + 1);
            out.push_back(list.substr(open + 1, close - open - 1));
            open = close;
        }
        return out;
    }

    std::string string(const std::string& key) {
        const size_t colon = after(key, ':');
        const size_t start = ok ? json.find('"', colon) : std::string::npos;
        const size_t end = start == std::string::npos ? start : json.find('"', start + 1);
        if (end == std::string::npos) {
            ok = false;
            return "";
        }
        at = end + 1;
        return json.substr(start + 1, end - start - 1);
    }

    float number(const std::string& key) {
        const size_t colon = after(key, ':');
        if (!ok) return 0.0f;
        char* end;
        const float value = std::strtof(json.c_str() + colon + 1, &end);
        at = end - json.c_str();
        return value;
    }
};

// An unreadable file yields no legs and no commands, which test_fixture_is_readable reports.
Fixture readFixture() {
    std::ifstream file("platform_shared/sign_convention.json");
    std::stringstream text;
    text << file.rdbuf();
    const std::string json = text.str();
    Reader r{json};
    Fixture f{};

    f.legs = r.strings("\"legs\"");
    const std::vector<float> stance = r.numbers("\"stance_mm\"", '[', ']', 18);
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) f.stance[i][j] = stance[i * 3 + j];
    }
    f.dt = r.number("\"dt_s\"");
    f.ticks = static_cast<int>(r.number("\"ticks\""));
    f.zeroMmPerTick = r.number("\"zero_mm_per_tick\"");
    f.zeroRadPerTick = r.number("\"zero_rad_per_tick\"");

    r.after("\"commands\"", '[');
    while (r.ok && r.has("\"name\"")) {
        Command c{};
        c.name = r.string("\"name\"");
        const std::vector<float> gait = r.numbers("\"gait\"", '{', '}', 3);
        c.stepX = gait[0];
        c.stepY = gait[1];
        c.stepAngle = gait[2];
        const std::vector<float> body = r.numbers("\"body_velocity_sign\"", '[', ']', 2);
        c.bodyVelocitySign[0] = static_cast<int>(body[0]);
        c.bodyVelocitySign[1] = static_cast<int>(body[1]);
        c.yawSign = static_cast<int>(r.number("\"yaw_sign\""));
        const std::vector<float> feet = r.numbers("\"stance_foot_displacement_sign\"", '[', ']', 12);
        for (int i = 0; i < 6; ++i) {
            c.footSign[i][0] = static_cast<int>(feet[i * 2]);
            c.footSign[i][1] = static_cast<int>(feet[i * 2 + 1]);
        }
        f.commands.push_back(c);
    }
    if (!r.ok || f.legs.size() != 6) return Fixture{};
    return f;
}

int sign(float value, float zero) { return value > zero ? 1 : (value < -zero ? -1 : 0); }

gait_state_t tripod(float stepX, float stepY, float stepAngle, float stepHeight) {
    gait_state_t gait{};
    gait.gait_type = GaitType::TRI_GATE;
    gait.step_x = stepX;
    gait.step_y = stepY;
    gait.step_angle = stepAngle;
    gait.step_speed = 1.0f;
    gait.step_height = stepHeight;
    gait.step_depth = 0.0f;
    return gait;
}

// gait.h's own resting feet: with no stroke, no lift and no depth, generateFeet places every foot
// exactly on its defaultPosition.
BodyStateMsg restingStance(GaitController& controller) {
    gait_state_t still = tripod(0.0f, 0.0f, 0.0f, 0.0f);
    controller.setGait(still);
    BodyStateMsg body{};
    controller.generateFeet(still, body);
    return body;
}

struct Walked {
    float footPerTick[6][2];  // mean stance-foot displacement per tick, body frame
    int stanceTicks[6];
    float bodyPerTick[2];  // body translation the stance feet imply
    float yawPerTick;      // body rotation the stance feet imply, positive counter-clockwise from above
};

// Walks the command on gait.h's resting stance for the fixture's ticks and fits the rigid body motion
// the planted feet imply: a planted foot moves against the body, so v_body = -mean(v_foot), and the
// yaw rate is the least-squares rotation of the feet about the body centre, negated.
Walked walk(const Fixture& f, const Command& c) {
    GaitController controller;
    BodyStateMsg body = restingStance(controller);
    float rest[6][2];
    for (int i = 0; i < 6; ++i) {
        rest[i][0] = body.feet[i][0];
        rest[i][1] = body.feet[i][1];
    }

    gait_state_t gait = tripod(c.stepX, c.stepY, c.stepAngle, 40.0f);
    gait.step_depth = 0.002f;
    controller.setGait(gait);
    controller.step(gait, body, f.dt);
    // A cycle started from rest carries the feet's offset off over FEET_EASE_S; the convention is a
    // steady walk's, so sampling starts once that has gone.
    const int settleTicks = static_cast<int>(std::ceil(FEET_EASE_S / f.dt));
    for (int tick = 0; tick < settleTicks; ++tick) controller.step(gait, body, f.dt);

    const auto inStance = [&](float phase, int leg) {
        return std::fmod(phase + gait.offset[leg], 1.0f) < gait.stand_frac;
    };

    float sum[6][2] = {};
    Walked w{};
    for (int tick = 0; tick < f.ticks; ++tick) {
        const float phaseBefore = controller.getPhase();
        float before[6][2];
        for (int i = 0; i < 6; ++i) {
            before[i][0] = body.feet[i][0];
            before[i][1] = body.feet[i][1];
        }
        controller.step(gait, body, f.dt);
        for (int i = 0; i < 6; ++i) {
            if (!inStance(phaseBefore, i) || !inStance(controller.getPhase(), i)) continue;
            sum[i][0] += body.feet[i][0] - before[i][0];
            sum[i][1] += body.feet[i][1] - before[i][1];
            w.stanceTicks[i]++;
        }
    }

    float cross = 0.0f, radiusSquared = 0.0f;
    for (int i = 0; i < 6; ++i) {
        const int n = w.stanceTicks[i] > 0 ? w.stanceTicks[i] : 1;
        w.footPerTick[i][0] = sum[i][0] / n;
        w.footPerTick[i][1] = sum[i][1] / n;
        w.bodyPerTick[0] -= w.footPerTick[i][0] / 6.0f;
        w.bodyPerTick[1] -= w.footPerTick[i][1] / 6.0f;
        cross += rest[i][0] * w.footPerTick[i][1] - rest[i][1] * w.footPerTick[i][0];
        radiusSquared += rest[i][0] * rest[i][0] + rest[i][1] * rest[i][1];
    }
    w.yawPerTick = -cross / radiusSquared;
    return w;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_fixture_is_readable() {
    const Fixture f = readFixture();
    TEST_ASSERT_EQUAL_MESSAGE(3, f.commands.size(), "sign_convention.json missing or unreadable");
    TEST_ASSERT_EQUAL(6, f.legs.size());
}

void test_resting_stance_puts_each_leg_on_its_fixture_side() {
    const Fixture f = readFixture();
    TEST_ASSERT_EQUAL_MESSAGE(6, f.legs.size(), "sign_convention.json missing or unreadable");
    GaitController controller;
    const BodyStateMsg rest = restingStance(controller);
    for (int i = 0; i < 6; ++i) {
        TEST_ASSERT_EQUAL_MESSAGE(sign(f.stance[i][0], 0.0f), sign(rest.feet[i][0], 0.0f), f.legs[i].c_str());
        TEST_ASSERT_EQUAL_MESSAGE(sign(f.stance[i][1], 0.0f), sign(rest.feet[i][1], 0.0f), f.legs[i].c_str());
    }
}

void test_kinematic_mounts_put_each_leg_on_its_fixture_side() {
    // The IK must drive the leg the gait places: same side, same front-to-rear slot, same index.
    const Fixture f = readFixture();
    TEST_ASSERT_EQUAL_MESSAGE(6, f.legs.size(), "sign_convention.json missing or unreadable");
    for (int i = 0; i < 6; ++i) {
        TEST_ASSERT_EQUAL_MESSAGE(sign(f.stance[i][0], 0.0f), sign(hexapodConfig.mountX[i], 0.0f), f.legs[i].c_str());
        TEST_ASSERT_EQUAL_MESSAGE(sign(f.stance[i][1], 0.0f), sign(hexapodConfig.mountY[i], 0.0f), f.legs[i].c_str());
    }
}

void test_stance_feet_move_with_the_fixture_signs() {
    const Fixture f = readFixture();
    TEST_ASSERT_EQUAL_MESSAGE(3, f.commands.size(), "sign_convention.json missing or unreadable");
    for (const Command& c : f.commands) {
        const Walked w = walk(f, c);
        for (int i = 0; i < 6; ++i) {
            const std::string where = c.name + ", " + f.legs[i];
            TEST_ASSERT_GREATER_THAN_MESSAGE(0, w.stanceTicks[i], where.c_str());
            TEST_ASSERT_EQUAL_MESSAGE(c.footSign[i][0], sign(w.footPerTick[i][0], f.zeroMmPerTick), where.c_str());
            TEST_ASSERT_EQUAL_MESSAGE(c.footSign[i][1], sign(w.footPerTick[i][1], f.zeroMmPerTick), where.c_str());
        }
    }
}

void test_body_motion_has_the_fixture_signs() {
    const Fixture f = readFixture();
    TEST_ASSERT_EQUAL_MESSAGE(3, f.commands.size(), "sign_convention.json missing or unreadable");
    for (const Command& c : f.commands) {
        const Walked w = walk(f, c);
        TEST_ASSERT_EQUAL_MESSAGE(c.bodyVelocitySign[0], sign(w.bodyPerTick[0], f.zeroMmPerTick), c.name.c_str());
        TEST_ASSERT_EQUAL_MESSAGE(c.bodyVelocitySign[1], sign(w.bodyPerTick[1], f.zeroMmPerTick), c.name.c_str());
        TEST_ASSERT_EQUAL_MESSAGE(c.yawSign, sign(w.yawPerTick, f.zeroRadPerTick), c.name.c_str());
    }
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_fixture_is_readable);
    RUN_TEST(test_resting_stance_puts_each_leg_on_its_fixture_side);
    RUN_TEST(test_kinematic_mounts_put_each_leg_on_its_fixture_side);
    RUN_TEST(test_stance_feet_move_with_the_fixture_signs);
    RUN_TEST(test_body_motion_has_the_fixture_signs);
    return UNITY_END();
}
