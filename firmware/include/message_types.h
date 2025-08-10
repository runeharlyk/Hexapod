#pragma once

// TODO: Find a way to match a topic with the data type
enum message_topic_t {
    SERVO_SIGNAL = 1,
    COMMAND = 2,
    MODE = 3,
    GAIT = 4,
    IMU = 5,
    ANGLE = 6,
    BODY = 7,
    WIFI = 8,
    SERVO_SETTINGS = 9,
    BLUETOOTH = 10,
    CAMERA = 11,
    AP = 12,
    DEVICE = 13,
    PERIPHERAL = 14
};

enum class MOTION_STATE { DEACTIVATED, IDLE, POSE, STAND, WALK };

struct ModeMsg {
    MOTION_STATE mode;
    friend void toJson(JsonVariant v, ModeMsg const &m) { v.set((int)m.mode); }
    void fromJson(JsonVariantConst o) { mode = (MOTION_STATE)o.as<int>(); }
};

enum class GaitType { TRI_GATE, BI_GATE, WAVE, RIPPLE };

struct GaitMsg {
    GaitType gait;
    friend void toJson(JsonVariant v, GaitMsg const &g) { v.set((int)g.gait); }
    void fromJson(JsonVariantConst o) { gait = (GaitType)o.as<int>(); }
};

struct ServoAnglesMsg {
    float angles[18];
    friend void toJson(JsonVariant v, ServoAnglesMsg const &a) {
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

struct ServoSignalMsg {
    int8_t id;
    uint16_t pwm;
    friend void toJson(JsonVariant v, ServoSignalMsg const &s) {
        v["id"] = s.id;
        v["pwm"] = s.pwm;
    }
    void fromJson(JsonVariantConst o) {
        id = o["id"].as<int8_t>();
        pwm = o["pwm"].as<uint16_t>();
    }
};

struct IMUAnglesMsg {
    float rpy[3];
    friend void toJson(JsonVariant v, IMUAnglesMsg const &a) {
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

struct CommandMsg {
    float lx, ly, rx, ry, h, s, s1;
    friend void toJson(JsonVariant v, CommandMsg const &c) {
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

struct ServoSetting {
    int16_t centerPwm;
    int8_t direction;
    uint8_t pin;
    float conversion;
    String name;
};

class ServoSettingsMsg {
  public:
    ServoSetting servos[NUM_SERVO] = {{306, 1, 8, 2, "LF1"}, {306, 1, 9, 2, "LF2"},  {306, 1, 10, 2, "LF3"},
                                      {306, 1, 2, 2, "LC1"}, {306, -1, 3, 2, "LC2"}, {306, -1, 4, 2, "LC3"},
                                      {306, 1, 5, 2, "LB1"}, {306, -1, 6, 2, "LB2"}, {306, -1, 7, 2, "LB3"},
                                      {306, 1, 8, 2, "RF1"}, {306, 1, 9, 2, "RF2"},  {306, 1, 10, 2, "RF3"},
                                      {306, 1, 2, 2, "RC1"}, {306, -1, 3, 2, "RC2"}, {306, -1, 4, 2, "RC3"},
                                      {306, 1, 5, 2, "RB1"}, {306, -1, 6, 2, "RB2"}, {306, -1, 7, 2, "RB3"}};

    friend void toJson(JsonVariant dst, ServoSettingsMsg const &src) {
        JsonArray arr = dst.to<JsonArray>();
        for (auto const &s : src.servos) {
            JsonObject o = arr.add<JsonObject>();
            o["center"] = s.centerPwm;
            o["dir"] = s.direction;
            o["pin"] = s.pin;
            o["conv"] = s.conversion;
            o["name"] = s.name;
        }
    }

    void fromJson(JsonVariantConst src) {
        JsonArrayConst arr = src.as<JsonArrayConst>();
        size_t i = 0;
        for (JsonObjectConst o : arr) {
            if (i >= NUM_SERVO) break;
            servos[i].centerPwm = o["center"] | servos[i].centerPwm;
            servos[i].direction = o["dir"] | servos[i].direction;
            servos[i].pin = o["pin"] | servos[i].pin;
            servos[i].conversion = o["conv"] | servos[i].conversion;
            servos[i].name = o["name"] | servos[i].name;
            ++i;
        }
    }
};

struct WiFiSettingsMsg {
    String ssid;
    String password;
    String hostname;

    friend void toJson(JsonVariant dst, WiFiSettingsMsg const &src) {
        dst["ssid"] = src.ssid;
        dst["password"] = src.password;
        dst["hostname"] = src.hostname;
    }

    void fromJson(JsonVariantConst src) {
        ssid = src["ssid"] | ssid;
        password = src["password"] | password;
        hostname = src["hostname"] | hostname;
    }
};
