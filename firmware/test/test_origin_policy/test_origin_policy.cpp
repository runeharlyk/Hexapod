// Host tests of the origin policy that guards the robot's WebSocket and its CORS answers: every page
// the app legitimately runs from must get through, and a page from the internet must not.

#include <unity.h>

#include <communication/origin_policy.h>

void setUp() {}
void tearDown() {}

// The embedded app, however the robot was reached: its IP, its mDNS name, or its access point.
void test_the_app_the_robot_serves_may_connect() {
    TEST_ASSERT_TRUE(originAllowed("http://192.168.1.39", "192.168.1.39"));
    TEST_ASSERT_TRUE(originAllowed("http://esp32.local", "esp32.local"));
    TEST_ASSERT_TRUE(originAllowed("http://Hexapod.local", "hexapod.local"));
    TEST_ASSERT_TRUE(originAllowed("http://192.168.4.1:80", "192.168.4.1:80"));
}

// pnpm dev and vite preview serve the app from the developer's machine, or to a phone with --host.
void test_the_app_served_from_the_local_network_may_connect() {
    TEST_ASSERT_TRUE(originAllowed("http://localhost:5173", "192.168.1.39"));
    TEST_ASSERT_TRUE(originAllowed("http://127.0.0.1:4173", "192.168.1.39"));
    TEST_ASSERT_TRUE(originAllowed("http://[::1]:5173", "192.168.1.39"));
    TEST_ASSERT_TRUE(originAllowed("https://localhost:5173", "192.168.1.39"));
    TEST_ASSERT_TRUE(originAllowed("http://192.168.1.23:5173", "192.168.1.39"));
    TEST_ASSERT_TRUE(originAllowed("http://10.0.0.7", "192.168.1.39"));
    TEST_ASSERT_TRUE(originAllowed("http://172.31.255.254:8080", "192.168.1.39"));
    TEST_ASSERT_TRUE(originAllowed("http://169.254.12.1", "192.168.1.39"));
    TEST_ASSERT_TRUE(originAllowed("http://192.168.4.2", "192.168.4.1"));
}

void test_the_hosted_app_may_connect() {
    TEST_ASSERT_TRUE(originAllowed("https://runeharlyk.github.io", "192.168.1.39"));
    TEST_ASSERT_TRUE(originAllowed("https://RuneHarlyk.github.io", "esp32.local"));
}

// Scripts and tools (robot_animate.py, the Python robot client) send no Origin; a page cannot omit it.
void test_a_client_without_an_origin_may_connect() { TEST_ASSERT_TRUE(originAllowed(nullptr, "192.168.1.39")); }

void test_any_other_page_may_not() {
    const char* refused[] = {"http://evil.example",
                             "https://runeharlyk.github.io.evil.example",
                             "https://evil.github.io",
                             "http://runeharlyk.github.io",
                             "null",
                             "",
                             "http://192.168.1.39.evil.example",
                             "http://localhost.evil.example",
                             "http://localhost:5173.evil.example",
                             "http://localhost:",
                             "ftp://192.168.1.39",
                             "http://172.32.0.1",
                             "http://11.0.0.1",
                             "http://192.169.1.1",
                             "http://10.evil.example",
                             "http://192.168.1.300",
                             "http://192.168.1",
                             "http://evil.local.example.com",
                             "http://.local",
                             "http://192.168.1.40:80x"};
    for (const char* origin : refused) TEST_ASSERT_FALSE_MESSAGE(originAllowed(origin, "192.168.1.39"), origin);
    TEST_ASSERT_FALSE(originAllowed("http://robot.example", nullptr));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_the_app_the_robot_serves_may_connect);
    RUN_TEST(test_the_app_served_from_the_local_network_may_connect);
    RUN_TEST(test_the_hosted_app_may_connect);
    RUN_TEST(test_a_client_without_an_origin_may_connect);
    RUN_TEST(test_any_other_page_may_not);
    return UNITY_END();
}
