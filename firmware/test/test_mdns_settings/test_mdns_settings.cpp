// Host tests of the mDNS settings check, which runs both on the settings file read at boot and on an
// update.

#include <unity.h>

#include <cstring>
#include <string>
#include <settings/mdns_settings.h>

namespace {

MDNSSettings stored(const char* hostname, const char* instance) {
    MDNSSettings settings = MDNSSettings_defaults();
    std::strncpy(settings.hostname, hostname, sizeof(settings.hostname) - 1);
    std::strncpy(settings.instance, instance, sizeof(settings.instance) - 1);
    return settings;
}

void addTxt(MDNSSettings& settings, const char* key, const char* value) {
    MDNSTxtRecord& txt = settings.global_txt_records[settings.global_txt_records_count++];
    std::strncpy(txt.key, key, sizeof(txt.key) - 1);
    std::strncpy(txt.value, value, sizeof(txt.value) - 1);
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_a_fresh_robot_advertises_as_the_hexapod() {
    const MDNSSettings settings = MDNSSettings_defaults();
    TEST_ASSERT_EQUAL_STRING("hexapod", settings.hostname);
    TEST_ASSERT_EQUAL_STRING("Hexapod", settings.instance);
    TEST_ASSERT_EQUAL(0, settings.global_txt_records_count);
}

// A robot first flashed before July 2026 kept the ESP32 template's names in its settings file.
void test_the_esp32_template_names_stored_by_earlier_firmware_are_replaced() {
    MDNSSettings file = stored("esp32", "ESP32 Device");
    MDNSSettings state = api_MDNSSettings_init_zero;
    TEST_ASSERT_EQUAL(StateUpdateResult::CHANGED, MDNSSettings_update(file, state));
    TEST_ASSERT_EQUAL_STRING("hexapod", state.hostname);
    TEST_ASSERT_EQUAL_STRING("Hexapod", state.instance);
}

void test_a_chosen_name_is_kept() {
    MDNSSettings file = stored("esp32-lab", "Lab hexapod");
    MDNSSettings state = api_MDNSSettings_init_zero;
    MDNSSettings_update(file, state);
    TEST_ASSERT_EQUAL_STRING("esp32-lab", state.hostname);
    TEST_ASSERT_EQUAL_STRING("Lab hexapod", state.instance);
}

void test_an_empty_hostname_falls_back_to_the_factory_name() {
    MDNSSettings file = stored("", "Lab hexapod");
    MDNSSettings state = api_MDNSSettings_init_zero;
    MDNSSettings_update(file, state);
    TEST_ASSERT_EQUAL_STRING("hexapod", state.hostname);
}

// Earlier firmware saved its own version as a TXT record, which then outlived every update: a robot
// running 0.1.1 advertised 0.0.1.
void test_a_stored_firmware_version_is_dropped_and_other_txt_records_kept() {
    MDNSSettings file = stored("hexapod", "Hexapod");
    addTxt(file, "Firmware Version", "0.0.1");
    addTxt(file, "owner", "lab");
    MDNSSettings state = api_MDNSSettings_init_zero;
    MDNSSettings_update(file, state);
    TEST_ASSERT_EQUAL(1, state.global_txt_records_count);
    TEST_ASSERT_EQUAL_STRING("owner", state.global_txt_records[0].key);
    TEST_ASSERT_EQUAL_STRING("lab", state.global_txt_records[0].value);
}

void test_migrated_settings_are_stable_on_the_next_boot() {
    MDNSSettings file = stored("esp32", "ESP32 Device");
    addTxt(file, "Firmware Version", "0.0.1");
    MDNSSettings state = api_MDNSSettings_init_zero;
    MDNSSettings_update(file, state);
    const MDNSSettings migrated = state;
    TEST_ASSERT_EQUAL(StateUpdateResult::UNCHANGED, MDNSSettings_update(migrated, state));
}

// A name the user enters is refused, not changed: the app shows the robot under the name it stored.
void test_a_hostname_must_be_one_dns_label() {
    TEST_ASSERT_TRUE(MDNSSettings_validHostname("hexapod-2"));
    TEST_ASSERT_TRUE(MDNSSettings_validHostname(std::string(32, 'a').c_str()));
    TEST_ASSERT_FALSE(MDNSSettings_validHostname(""));
    TEST_ASSERT_FALSE(MDNSSettings_validHostname("-a"));
    TEST_ASSERT_FALSE(MDNSSettings_validHostname("a-"));
    TEST_ASSERT_FALSE(MDNSSettings_validHostname("a.b"));
    TEST_ASSERT_FALSE(MDNSSettings_validHostname("Hexapod"));
    TEST_ASSERT_FALSE(MDNSSettings_validHostname(std::string(33, 'a').c_str()));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_a_fresh_robot_advertises_as_the_hexapod);
    RUN_TEST(test_the_esp32_template_names_stored_by_earlier_firmware_are_replaced);
    RUN_TEST(test_a_chosen_name_is_kept);
    RUN_TEST(test_an_empty_hostname_falls_back_to_the_factory_name);
    RUN_TEST(test_a_stored_firmware_version_is_dropped_and_other_txt_records_kept);
    RUN_TEST(test_migrated_settings_are_stable_on_the_next_boot);
    RUN_TEST(test_a_hostname_must_be_one_dns_label);
    return UNITY_END();
}
