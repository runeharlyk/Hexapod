#pragma once

// TODO: Find a way to match a topic with the data type
enum message_topic_t { TEMP = 1, COMMAND = 2, MODE = 3, GAIT = 4, IMU = 5, ANGLE = 6, BODY = 7 };

struct Temp {
    float value;
    friend void toJson(JsonVariant v, Temp const &t) { v["value"] = t.value; }
    void fromJson(JsonObjectConst o) { value = o["value"].as<float>(); }
};

enum class MOTION_STATE { DEACTIVATED, IDLE, POSE, STAND, WALK };

struct Mode {
    MOTION_STATE mode;
    friend void toJson(JsonVariant v, Mode const &m) { v.set((int)m.mode); }
    void fromJson(JsonVariantConst o) { mode = (MOTION_STATE)o.as<int>(); }
};

enum class GaitType { TRI_GATE, BI_GATE, WAVE, RIPPLE };

struct Gait {
    GaitType gait;
    friend void toJson(JsonVariant v, Gait const &g) { v.set((int)g.gait); }
    void fromJson(JsonVariantConst o) { gait = (GaitType)o.as<int>(); }
};

struct ServoAngles {
    float angles[18];
    friend void toJson(JsonVariant v, ServoAngles const &a) {
        JsonArray arr = v.to<JsonArray>();
        for (int i = 0; i < 12; i++) {
            arr.add(a.angles[i]);
        }
    }
    void fromJson(JsonVariantConst o) {
        JsonArrayConst arr = o.as<JsonArrayConst>();
        for (int i = 0; i < 12; i++) {
            angles[i] = arr[i].as<float>();
        }
    }
};

struct IMUAngles {
    float rpy[3];
    friend void toJson(JsonVariant v, IMUAngles const &a) {
        JsonArray arr = v.to<JsonArray>();
        for (int i = 0; i < 3; i++) {
            arr.add(a.rpy[i]);
        }
    }
    void fromJson(JsonVariantConst o) {
        JsonArrayConst arr = o.as<JsonArrayConst>();
        for (int i = 0; i < 3; i++) {
            rpy[i] = arr[i].as<float>();
        }
    }
};

struct Command {
    float lx, ly, rx, ry, h, s, s1;
    friend void toJson(JsonVariant v, Command const &c) {
        JsonArray arr = v.to<JsonArray>();
        arr.add(static_cast<int8_t>(c.lx * 127));
        arr.add(static_cast<int8_t>(c.ly * 127));
        arr.add(static_cast<int8_t>(c.rx * 127));
        arr.add(static_cast<int8_t>(c.ry * 127));
        arr.add(static_cast<int8_t>(c.h * 127));
        arr.add(static_cast<int8_t>(c.s * 127));
        arr.add(static_cast<int8_t>(c.s1 * 127));
    }

    void fromJson(JsonVariantConst o) {
        JsonArrayConst arr = o.as<JsonArrayConst>();
        lx = arr[0].as<int8_t>() / 127.0f;
        ly = arr[1].as<int8_t>() / 127.0f;
        rx = arr[2].as<int8_t>() / 127.0f;
        ry = arr[3].as<int8_t>() / 127.0f;
        h = arr[4].as<int8_t>() / 127.0f;
        s = arr[5].as<int8_t>() / 127.0f;
        s1 = arr[6].as<int8_t>() / 127.0f;
    }
};
