# Main component. Hand-written sources are listed explicitly (not globbed) so the IDF
# migration can proceed incrementally: Arduino .cpp files still on disk stay out of the
# build until each is ported. Generated nanopb sources are globbed from platform_shared/,
# which firmware/scripts/compile_protos.py regenerates via the pre_build.py pre-script.
file(GLOB PROTO_SRCS "platform_shared/*.pb.c")

set(SRCS
    "main.cpp"
    "features.cpp"
    "filesystem.cpp"
    "wifi/wifi_idf.cpp"
    "wifi_service.cpp"
    "ap_service.cpp"
    "mdns_service.cpp"
    "communication/webserver.cpp"
    "communication/websocket.cpp"
    "communication/ble.cpp"
    "www_mount.cpp"
    "../../submodules/nanopb/pb_encode.c"
    "../../submodules/nanopb/pb_decode.c"
    "../../submodules/nanopb/pb_common.c"
    ${PROTO_SRCS}
)

set(COMPONENT_REQUIRES
    nvs_flash
    esp_timer
    esp_wifi
    esp_netif
    esp_event
    esp_http_server
    esp_driver_i2c
    json
    lwip
    joltwallet__littlefs
    espressif__mdns
    espressif__esp-dsp
    esp-nimble-cpp
    espressif__esp32-camera
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "../include" "platform_shared" "../../submodules/nanopb"
    REQUIRES ${COMPONENT_REQUIRES}
)
