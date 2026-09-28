#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_chip_info.h>
#include <esp_heap_caps.h>
#include <esp_idf_version.h>
#include <esp_flash.h>
#include <esp_sleep.h>
#include <esp_littlefs.h>
#include <driver/temperature_sensor.h>
#include <nvs_flash.h>
#include <dirent.h>
#include <unistd.h>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <features.h>
#include <communication/espnow_adapter.h>
#include <ota_service.h>
#include <peripherals_settings_service.h>
#include <filesystem.h>
#include <wifi/wifi_idf.h>
#include <wifi_service.h>
#include <ap_service.h>
#include <mdns_service.h>
#include <communication/webserver.h>
#include <communication/websocket.h>
#include <communication/ble.h>
#include <communication/serial_adapter.h>
#include <www_mount.hpp>
#include <hexapod.h>
#include <cpu_stats.h>
#include <filesystem_service.h>
#include <camera_service.h>
#include <servo_settings_service.h>
#include <event_bus.h>
#include <message_types.h>
#include <platform_shared/message.pb.h>

static const char *TAG = "main";

// Internal DMA RAM is the binding constraint here, and total free heap hides it -- PSRAM dominates.
static void logHeap(const char *stage) {
    ESP_LOGW(TAG, "HEAP %-18s internal=%6u largestInt=%6u DMA=%6u largestDMA=%6u", stage,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
}

WiFiService wifiService;
APService apService;
MDNSService mdnsService;
ServoSettingsService servoSettingsService;
PeripheralSettingsService peripheralSettingsService;
Websocket wsSocket{server, "/api/ws"};
BLE bleAdapter;
#if FT_ENABLED(USE_SERIAL_LINK)
SerialAdapter serialAdapter;
#endif
Hexapod robot;
#if FT_ENABLED(USE_ESPNOW)
EspNowAdapter espNow;
#endif
CpuMonitor cpuMonitor;
static temperature_sensor_handle_t coreTempSensor = nullptr;

// Control-loop iterations since boot. The OTA image is only confirmed once this shows the loop has
// been running for a while, not merely that app_main was reached.
static std::atomic<uint32_t> controlTicks{0};
static constexpr uint32_t TICKS_BEFORE_IMAGE_CONFIRM = 200;  // 1 s of the 5 ms loop

static void initCoreTempSensor() {
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&cfg, &coreTempSensor) != ESP_OK ||
        temperature_sensor_enable(coreTempSensor) != ESP_OK) {
        ESP_LOGW(TAG, "Core temperature sensor unavailable");
        coreTempSensor = nullptr;
    }
}

static const char *resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON: return "power-on";
        case ESP_RST_EXT: return "external pin";
        case ESP_RST_SW: return "software restart";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "interrupt watchdog";
        case ESP_RST_TASK_WDT: return "task watchdog";
        case ESP_RST_WDT: return "other watchdog";
        case ESP_RST_DEEPSLEEP: return "deep-sleep wake";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO: return "SDIO";
        case ESP_RST_USB: return "USB peripheral";
        case ESP_RST_JTAG: return "JTAG";
        case ESP_RST_EFUSE: return "efuse error";
        case ESP_RST_PWR_GLITCH: return "power glitch";
        case ESP_RST_CPU_LOCKUP: return "CPU lockup";
        default: return "unknown";
    }
}

static void fillAnalytics(socket_message_AnalyticsData &a) {
    // Internal RAM only, like total_heap: PSRAM would swamp the figure and hide internal exhaustion.
    a.free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    a.min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    a.total_heap = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    a.max_alloc_heap = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    a.psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    a.free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    a.uptime = esp_timer_get_time() / 1000000;
    size_t total = 0, used = 0;
    if (esp_littlefs_info(FS_PARTITION_LABEL, &total, &used) == ESP_OK) {
        a.fs_total = (int)total;
        a.fs_used = (int)used;
    }
    CpuMonitor::Usage cpu = cpuMonitor.latest();
    a.cpu0_usage = cpu.core0;
    a.cpu1_usage = cpu.core1;
    a.cpu_usage = cpu.total;
    float celsius = 0.0f;
    if (coreTempSensor && temperature_sensor_get_celsius(coreTempSensor, &celsius) == ESP_OK) a.core_temp = celsius;
}

static void fillStaticInfo(socket_message_StaticSystemInformation &s) {
    strncpy(s.esp_platform, CONFIG_IDF_TARGET, sizeof(s.esp_platform) - 1);
    strncpy(s.cpu_type, CONFIG_IDF_TARGET, sizeof(s.cpu_type) - 1);
    strncpy(s.firmware_version, APP_VERSION, sizeof(s.firmware_version) - 1);
    strncpy(s.sdk_version, esp_get_idf_version(), sizeof(s.sdk_version) - 1);
    s.cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_chip_info_t info;
    esp_chip_info(&info);
    s.cpu_cores = info.cores;
    uint32_t flash_size = 0;
    esp_flash_get_size(nullptr, &flash_size);
    s.flash_chip_size = flash_size;
    strncpy(s.cpu_reset_reason, resetReasonName(esp_reset_reason()), sizeof(s.cpu_reset_reason) - 1);
}

static constexpr size_t WIFI_SCAN_RESULT_CAPACITY = 20;

// Settings are persisted as binary protobuf under FS_CONFIG_DIRECTORY (the files keep their legacy
// .json names), so erasing NVS alone leaves WiFi credentials and servo calibration in place after a
// factory reset.
static void eraseConfigDirectory() {
    DIR *dir = opendir(FS_CONFIG_DIRECTORY);
    if (!dir) return;
    for (struct dirent *entry = readdir(dir); entry; entry = readdir(dir)) {
        if (entry->d_type == DT_DIR) continue;
        std::string path = std::string(FS_CONFIG_DIRECTORY "/") + entry->d_name;
        if (unlink(path.c_str()) != 0) ESP_LOGW(TAG, "Failed to erase %s", path.c_str());
    }
    closedir(dir);
}

// A copy taken under the service lock: the correlation handlers run on the adapter tasks while the
// service task may be updating the same state.
// Written straight into the response rather than returned: the correlation handler already holds a
// ~2 KB response on its stack, and a temporary settings copy would push it past the stack budget.
template <typename T>
static void snapshot(StatefulService<T> &service, T &out) {
    service.read([&out](const T &state) { out = state; });
}

// Passwords never leave the device. The update paths treat an empty password as "keep the stored
// one" (WiFiSettings_update / APSettings_update), so a client can round-trip these unchanged.
static void redactedWifiSettings(api_WifiSettings &out) {
    snapshot(wifiService, out);
    for (pb_size_t i = 0; i < out.wifi_networks_count; i++) {
        memset(out.wifi_networks[i].password, 0, sizeof(out.wifi_networks[i].password));
    }
}

static void redactedApSettings(api_APSettings &out) {
    snapshot(apService, out);
    memset(out.password, 0, sizeof(out.password));
}

// Drops the servos before the chip goes down: the PCA9685 keeps its last outputs across an ESP reset
// and deep sleep, so a restart would otherwise leave the legs energised while the firmware reports
// DEACTIVATED. The mode is applied on the EventBus worker, so give it a moment to reach the driver.
static void deactivateServos() {
    EventBus<ModeMsg>::publish({MOTION_STATE::DEACTIVATED});
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void registerHandlers(CommAdapterBase &c) {
    c.on<socket_message_ControllerInputData>([](const socket_message_ControllerInputData &in, int) {
        CommandMsg cmd{in.left.x, in.left.y, in.right.x, in.right.y, in.height, in.speed, in.s1, in.feet_distance};
        if (cmd.sanitize()) EventBus<CommandMsg>::publish(cmd);
    });
    c.on<socket_message_ModeData>([](const socket_message_ModeData &m, int) {
        if ((int)m.mode < 0 || (int)m.mode > (int)socket_message_ModesEnum_WALK_NN) return;
        if (!FT_ENABLED(USE_POLICY) && m.mode == socket_message_ModesEnum_WALK_NN) return;
        EventBus<ModeMsg>::publish({static_cast<MOTION_STATE>(m.mode)});
    });
    c.on<socket_message_GaitData>([](const socket_message_GaitData &g, int) {
        if ((int)g.gait < 0 || (int)g.gait > (int)socket_message_GaitEnum_AUTO) return;
        EventBus<GaitMsg>::publish({static_cast<GaitType>(g.gait)});
    });
    c.on<socket_message_ServoPWMData>([](const socket_message_ServoPWMData &s, int) {
        EventBus<ServoSignalMsg>::publish({static_cast<int8_t>(s.servo_id), static_cast<uint16_t>(s.servo_pwm)});
    });
    c.on<socket_message_ServoStateData>(
        [](const socket_message_ServoStateData &s, int) { EventBus<ServoStateMsg>::publish({s.active}); });

    c.on<socket_message_SystemCommandData>([](const socket_message_SystemCommandData &cmd, int) {
        switch (cmd.command) {
            case socket_message_SystemCommand_SYS_RESTART:
                deactivateServos();
                esp_restart();
                break;
            case socket_message_SystemCommand_SYS_RESET:
                deactivateServos();
                eraseConfigDirectory();
                nvs_flash_erase();
                esp_restart();
                break;
            case socket_message_SystemCommand_SYS_SLEEP:
                deactivateServos();
                esp_deep_sleep_start();
                break;
            default: break;
        }
    });

    // Owned per adapter: the response holds a pointer into it until adapter->emit returns, and the
    // websocket and BLE decoders run on separate tasks, so a single shared buffer would tear.
    auto scanBuffer = std::make_shared<std::array<api_WifiNetworkScan, WIFI_SCAN_RESULT_CAPACITY>>();
    c.on<socket_message_CorrelationRequest>([adapter = &c, scanBuffer](const socket_message_CorrelationRequest &req,
                                                                      int clientId) {
        socket_message_CorrelationResponse res = socket_message_CorrelationResponse_init_zero;
        res.correlation_id = req.correlation_id;
        res.status_code = 200;
        switch (req.which_request) {
            case socket_message_CorrelationRequest_features_data_request_tag: {
                res.which_response = socket_message_CorrelationResponse_features_data_response_tag;
                auto &f = res.response.features_data_response;
                strncpy(f.firmware_name, APP_NAME, sizeof(f.firmware_name) - 1);
                strncpy(f.firmware_version, APP_VERSION, sizeof(f.firmware_version) - 1);
                f.camera = USE_CAMERA;
                f.imu = USE_MPU6050;
                f.mag = robot.magnetometerPresent();
                f.servo = true;
                f.mdns = USE_MDNS;
                break;
            }
            case socket_message_CorrelationRequest_system_information_request_tag: {
                res.which_response = socket_message_CorrelationResponse_system_information_response_tag;
                auto &sys = res.response.system_information_response;
                sys.has_analytics_data = true;
                sys.has_static_system_information = true;
                fillAnalytics(sys.analytics_data);
                fillStaticInfo(sys.static_system_information);
                break;
            }
            case socket_message_CorrelationRequest_i2c_scan_data_request_tag: {
                res.which_response = socket_message_CorrelationResponse_i2c_scan_data_tag;
                auto &scan = res.response.i2c_scan_data;
                scan.devices_count = 0;
                for (uint8_t addr : robot.scanI2C()) {
                    if (scan.devices_count >= 16) break;
                    scan.devices[scan.devices_count++].address = addr;
                }
                break;
            }
            case socket_message_CorrelationRequest_wifi_settings_get_tag: {
                res.which_response = socket_message_CorrelationResponse_wifi_settings_tag;
                redactedWifiSettings(res.response.wifi_settings);
                break;
            }
            case socket_message_CorrelationRequest_wifi_settings_update_tag: {
                StateUpdateResult r = wifiService.update(
                    [&req](WiFiSettings &s) { return WiFiSettings_update(req.request.wifi_settings_update, s); },
                    "correlation");
                if (r == StateUpdateResult::ERROR) res.status_code = 400;
                res.which_response = socket_message_CorrelationResponse_wifi_settings_tag;
                redactedWifiSettings(res.response.wifi_settings);
                break;
            }
            case socket_message_CorrelationRequest_wifi_networks_get_tag: {
                api_WifiNetworkScan *scanBuf = scanBuffer->data();
                int count = WiFiService::networksProto(scanBuf, WIFI_SCAN_RESULT_CAPACITY);
                res.which_response = socket_message_CorrelationResponse_wifi_network_list_tag;
                res.response.wifi_network_list.networks = scanBuf;
                res.response.wifi_network_list.networks_count = count < 0 ? 0 : count;
                if (count == -1) res.status_code = 202;       // scan running; client polls again
                else if (count == -2) res.status_code = 503;  // radio busy (STA connecting); give up
                break;
            }
            case socket_message_CorrelationRequest_wifi_status_get_tag: {
                res.which_response = socket_message_CorrelationResponse_wifi_status_tag;
                WiFiService::statusProto(res.response.wifi_status);
                break;
            }
            case socket_message_CorrelationRequest_ap_settings_get_tag: {
                res.which_response = socket_message_CorrelationResponse_ap_settings_tag;
                redactedApSettings(res.response.ap_settings);
                break;
            }
            case socket_message_CorrelationRequest_ap_settings_update_tag: {
                StateUpdateResult r = apService.update(
                    [&req](APSettings &s) { return APSettings_update(req.request.ap_settings_update, s); },
                    "correlation");
                if (r == StateUpdateResult::ERROR) res.status_code = 400;
                res.which_response = socket_message_CorrelationResponse_ap_settings_tag;
                redactedApSettings(res.response.ap_settings);
                break;
            }
            case socket_message_CorrelationRequest_ap_status_get_tag: {
                res.which_response = socket_message_CorrelationResponse_ap_status_tag;
                apService.statusProto(res.response.ap_status);
                break;
            }
            case socket_message_CorrelationRequest_mdns_status_get_tag: {
                res.which_response = socket_message_CorrelationResponse_mdns_status_tag;
                mdnsService.statusProto(res.response.mdns_status);
                break;
            }
            case socket_message_CorrelationRequest_mdns_query_tag: {
                res.which_response = socket_message_CorrelationResponse_mdns_query_response_tag;
                mdnsService.queryProto(req.request.mdns_query, res.response.mdns_query_response);
                break;
            }
            case socket_message_CorrelationRequest_servo_settings_get_tag: {
                res.which_response = socket_message_CorrelationResponse_servo_settings_tag;
                snapshot(servoSettingsService, res.response.servo_settings);
                break;
            }
            case socket_message_CorrelationRequest_servo_settings_update_tag: {
                servoSettingsService.update(
                    [&req](ServoSettings &s) { return ServoSettings_update(req.request.servo_settings_update, s); },
                    "correlation");
                res.which_response = socket_message_CorrelationResponse_servo_settings_tag;
                snapshot(servoSettingsService, res.response.servo_settings);
                break;
            }
            case socket_message_CorrelationRequest_peripheral_settings_get_tag: {
                res.which_response = socket_message_CorrelationResponse_peripheral_settings_tag;
                snapshot(peripheralSettingsService, res.response.peripheral_settings);
                break;
            }
            case socket_message_CorrelationRequest_peripheral_settings_update_tag: {
                StateUpdateResult r = peripheralSettingsService.update(
                    [&req](PeripheralSettings &s) {
                        return PeripheralSettings_update(req.request.peripheral_settings_update, s);
                    },
                    "correlation");
                if (r == StateUpdateResult::ERROR) res.status_code = 400;
                res.which_response = socket_message_CorrelationResponse_peripheral_settings_tag;
                snapshot(peripheralSettingsService, res.response.peripheral_settings);
                break;
            }
            default: res.status_code = 400; break;
        }
        adapter->emit(res, clientId);
    });
}

template <typename T>
static void emitAll(const T &msg) {
    wsSocket.emit(msg);
    bleAdapter.emit(msg);
#if FT_ENABLED(USE_SERIAL_LINK)
    serialAdapter.emit(msg);
#endif
}

// A bridge holds its EventBus subscription only while some client is listening for that tag, so
// producers can skip work nobody will receive. Subscription changes arrive from every adapter task,
// so the registry and each bridge's check-then-subscribe run under one lock.
static std::mutex bridgeMutex;

static std::map<int32_t, std::function<void(bool)>> &bridges() {
    static std::map<int32_t, std::function<void(bool)>> registry;
    return registry;
}

static bool anyoneListening(int32_t tag) {
    return wsSocket.hasSubscribers(tag) || bleAdapter.hasSubscribers(tag)
#if FT_ENABLED(USE_SERIAL_LINK)
           || serialAdapter.hasSubscribers(tag)
#endif
        ;
}

static void refreshBridge(int32_t tag) {
    std::lock_guard<std::mutex> lock(bridgeMutex);
    auto it = bridges().find(tag);
    if (it != bridges().end()) it->second(anyoneListening(tag));
}

template <typename Msg, typename Fn>
static void addBridge(int32_t tag, Fn fn) {
    static typename EventBus<Msg>::Handle handle;
    std::lock_guard<std::mutex> lock(bridgeMutex);
    bridges()[tag] = [fn, tag](bool wanted) {
        if (wanted == handle.valid()) return;
        ESP_LOGD(TAG, "bridge tag %d %s", (int)tag, wanted ? "attached" : "detached");
        if (wanted) handle = EventBus<Msg>::subscribe(fn);
        else handle.unsubscribe();
    };
}

template <typename ProtoT>
static void observeStatus() {
    addBridge<ProtoT>(MessageTraits<ProtoT>::tag, [](const ProtoT &s) { emitAll(s); });
}

static void initNvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void setupServer() {
    server.config(16, 8192);
    server.listen(80);

    // Answer the browser's CORS preflight so the cross-origin dev app can reach the robot.
    server.addDefaultHeader("Access-Control-Allow-Origin", "*");
    server.addDefaultHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    server.addDefaultHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
    server.on("/*", HTTP_OPTIONS, [](httpd_req_t *r) {
        httpd_resp_set_status(r, "200 OK");
        return httpd_resp_send(r, nullptr, 0);
    });

    server.on("/api/features", HTTP_GET, [](httpd_req_t *r) {
        char buf[384];
        snprintf(buf, sizeof(buf),
                 "{\"camera\":%s,\"imu\":%s,\"mag\":%s,\"bmp\":false,\"mdns\":%s,\"servo\":true,"
                 "\"analytics\":true,\"sleep\":true,\"ota\":true,\"download_firmware\":true,"
                 "\"upload_firmware\":true,"
                 "\"firmware_version\":\"%s\",\"firmware_name\":\"%s\",\"firmware_built_target\":\"esp32-wroom-camera\"}",
                 USE_CAMERA ? "true" : "false", USE_MPU6050 ? "true" : "false",
                 robot.magnetometerPresent() ? "true" : "false",
                 USE_MDNS ? "true" : "false", APP_VERSION, APP_NAME);
        httpd_resp_set_type(r, "application/json");
        return httpd_resp_sendstr(r, buf);
    });

    fs_api::registerRoutes(server);
    ota_service::registerRoutes(server);
#if USE_CAMERA
    camera_service::registerRoutes(server);
#endif

    mountWebApp(server);
}

static void setupComm() {
    registerHandlers(wsSocket);
    registerHandlers(bleAdapter);
#if FT_ENABLED(USE_SERIAL_LINK)
    registerHandlers(serialAdapter);
    serialAdapter.onSubscriptionChange(refreshBridge);
#endif

    wsSocket.onSubscriptionChange(refreshBridge);
    bleAdapter.onSubscriptionChange(refreshBridge);

    addBridge<ServoAnglesMsg>(MessageTraits<socket_message_AnglesData>::tag, [](const ServoAnglesMsg &a) {
        socket_message_AnglesData out = socket_message_AnglesData_init_zero;
        out.angles_count = NUM_SERVO;
        for (int i = 0; i < NUM_SERVO; i++) out.angles[i] = static_cast<int32_t>(lroundf(a.angles[i]));
        emitAll(out);
    });

    addBridge<IMUAnglesMsg>(MessageTraits<socket_message_IMUData>::tag, [](const IMUAnglesMsg &m) {
        socket_message_IMUData out = socket_message_IMUData_init_zero;
        out.x = m.rpy[0];
        out.y = m.rpy[1];
        out.z = m.rpy[2];
        out.heading = m.heading;
        emitAll(out);
    });

    // Mode and gait also change from the ESP-NOW controller and from other clients, so every
    // change is pushed rather than assumed to originate from the viewing app.
    addBridge<ModeMsg>(MessageTraits<socket_message_ModeData>::tag, [](const ModeMsg &m) {
        socket_message_ModeData out = socket_message_ModeData_init_zero;
        out.mode = static_cast<socket_message_ModesEnum>(m.mode);
        emitAll(out);
    });

    addBridge<GaitMsg>(MessageTraits<socket_message_GaitData>::tag, [](const GaitMsg &g) {
        socket_message_GaitData out = socket_message_GaitData_init_zero;
        out.gait = static_cast<socket_message_GaitEnum>(g.gait);
        emitAll(out);
    });

    observeStatus<socket_message_OtaStatusData>();
    observeStatus<api_WifiStatus>();
    observeStatus<api_APStatus>();

    // Transports start only once every bridge exists, so an early subscription finds its bridge.
#if FT_ENABLED(USE_SERIAL_LINK)
    serialAdapter.begin();
#endif
    wsSocket.begin();

    bleAdapter.begin();
}

static void controlLoop(void *) {
    robot.initialize();
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        robot.readSensors();
        robot.planMotion();
        robot.updateActuators();
        robot.emitTelemetry();
        controlTicks.fetch_add(1, std::memory_order_relaxed);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
    }
}

static void serviceLoop(void *) {
    WiFi.init();
    wifiService.begin();
    mdnsService.begin();
    apService.begin();

    // Claim the AP's internal DMA buffers before httpd/BLE/ESP-NOW/camera take theirs: those degrade
    // cleanly when starved, ieee80211_hostap_attach null-derefs.
    apService.loop();
    logHeap("softap");

    setupServer();
    setupComm();

#if FT_ENABLED(USE_ESPNOW)
    espNow.begin();  // needs the radio started, so after wifiService/apService
#endif

    // After BLE so the controller claims its internal DMA first; camera init is non-fatal.
#if USE_CAMERA
    camera_service::init();
    logHeap("camera");
#endif

    ESP_LOGI(TAG, "Networking up, free heap %lu bytes", (unsigned long)esp_get_free_heap_size());

    bool imageConfirmed = false;
    for (;;) {
        // Rollback protection: only an image that brought up networking and kept the control loop
        // running is marked valid; one that crashes before this point reverts on the next reset.
        if (!imageConfirmed && controlTicks.load(std::memory_order_relaxed) >= TICKS_BEFORE_IMAGE_CONFIRM) {
            ota_service::confirmRunningImage();
            imageConfirmed = true;
        }
        wifiService.loop();
        apService.loop();
        EXECUTE_EVERY_N_MS(2000, {
            // sample() keeps running regardless: its uint32 idle-counter deltas only stay valid
            // over short windows. fillAnalytics() stats the filesystem, so that waits for a client.
            cpuMonitor.sample();
            if (anyoneListening(MessageTraits<socket_message_AnalyticsData>::tag)) {
                socket_message_AnalyticsData a = socket_message_AnalyticsData_init_zero;
                fillAnalytics(a);
                emitAll(a);
            }
        });
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "Booting %s (ESP-IDF)", APP_NAME);
    esp_log_level_set("NimBLE", ESP_LOG_WARN);  // NimBLE logs every notify at INFO — a flood
    initNvs();

    if (!FileSystem::init()) {
        ESP_LOGE(TAG, "Filesystem mount failed");
    }
    feature_service::printFeatureConfiguration();
#if FT_ENABLED(USE_POLICY)
    PolicyRunner::verify();
#endif
    initCoreTempSensor();
    servoSettingsService.begin();
    peripheralSettingsService.begin();

    xTaskCreate(serviceLoop, "service", 8192, nullptr, 2, nullptr);
    xTaskCreatePinnedToCore(controlLoop, "control", 8192, nullptr, 5, nullptr, 1);
}
