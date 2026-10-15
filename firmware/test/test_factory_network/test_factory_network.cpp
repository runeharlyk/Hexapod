// Host tests of merging the secrets.h network into the stored WiFi settings at boot.

#include <unity.h>

#include <cstring>
#include <initializer_list>
#include <settings/factory_network.h>

namespace {

api_WifiSettings withNetworks(std::initializer_list<const char*> ssids) {
    api_WifiSettings settings = api_WifiSettings_init_zero;
    for (const char* ssid : ssids) {
        api_WifiNetwork& n = settings.wifi_networks[settings.wifi_networks_count++];
        std::strncpy(n.ssid, ssid, sizeof(n.ssid) - 1);
        std::strncpy(n.password, "stored", sizeof(n.password) - 1);
    }
    return settings;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_a_new_secret_network_is_added_once() {
    api_WifiSettings settings = withNetworks({"office"});
    TEST_ASSERT_TRUE(applyFactoryNetwork(settings, "home", "secret123"));
    TEST_ASSERT_EQUAL(2, settings.wifi_networks_count);
    TEST_ASSERT_EQUAL_STRING("home", settings.wifi_networks[1].ssid);
    TEST_ASSERT_EQUAL_STRING("secret123", settings.wifi_networks[1].password);
    // Every later boot with the same secret leaves the settings alone.
    TEST_ASSERT_FALSE(applyFactoryNetwork(settings, "home", "secret123"));
    TEST_ASSERT_EQUAL(2, settings.wifi_networks_count);
}

void test_a_network_deleted_in_the_app_stays_deleted_until_the_secret_changes() {
    api_WifiSettings settings = withNetworks({});
    applyFactoryNetwork(settings, "home", "secret123");
    settings.wifi_networks_count = 0;  // the user removed it
    TEST_ASSERT_FALSE(applyFactoryNetwork(settings, "home", "secret123"));
    TEST_ASSERT_EQUAL(0, settings.wifi_networks_count);
    TEST_ASSERT_TRUE(applyFactoryNetwork(settings, "home", "new-password"));
    TEST_ASSERT_EQUAL(1, settings.wifi_networks_count);
}

void test_a_changed_password_updates_the_stored_network_of_that_name() {
    api_WifiSettings settings = withNetworks({"home", "office"});
    TEST_ASSERT_TRUE(applyFactoryNetwork(settings, "home", "rotated1"));
    TEST_ASSERT_EQUAL(2, settings.wifi_networks_count);
    TEST_ASSERT_EQUAL_STRING("rotated1", settings.wifi_networks[0].password);
    TEST_ASSERT_EQUAL_STRING("stored", settings.wifi_networks[1].password);
}

void test_an_empty_secret_adds_nothing() {
    api_WifiSettings settings = withNetworks({"office"});
    TEST_ASSERT_FALSE(applyFactoryNetwork(settings, "", ""));
    TEST_ASSERT_EQUAL(1, settings.wifi_networks_count);
    TEST_ASSERT_EQUAL_UINT32(0, settings.applied_secret);
}

void test_a_full_list_is_left_alone_and_tried_again_later() {
    api_WifiSettings settings = withNetworks({"a", "b", "c", "d", "e"});
    TEST_ASSERT_FALSE(applyFactoryNetwork(settings, "home", "secret123"));
    TEST_ASSERT_EQUAL_UINT32(0, settings.applied_secret);
    settings.wifi_networks_count = 4;
    TEST_ASSERT_TRUE(applyFactoryNetwork(settings, "home", "secret123"));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_a_new_secret_network_is_added_once);
    RUN_TEST(test_a_network_deleted_in_the_app_stays_deleted_until_the_secret_changes);
    RUN_TEST(test_a_changed_password_updates_the_stored_network_of_that_name);
    RUN_TEST(test_an_empty_secret_adds_nothing);
    RUN_TEST(test_a_full_list_is_left_alone_and_tried_again_later);
    return UNITY_END();
}
