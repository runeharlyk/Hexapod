// Host test of the nanopb struct sizes. Every adapter keeps two Messages (its encoder's and its
// decoder's) and every correlation request builds its CorrelationResponse on the adapter task's
// stack, so a new max_size or max_count in the .options files lands in internal RAM several times
// over. The limits sit just above today's sizes, which are the same on the host and the ESP32-S3;
// raising one is a decision to be taken with the RAM budget in view, not a test to quieten.

#include <unity.h>

#include <cstring>
#include <iterator>
#include <pb_encode.h>
#include <communication/proto_helpers.h>
#include <platform_shared/message.pb.h>

void setUp() {}
void tearDown() {}

void test_the_protocol_structs_stay_small() {
    TEST_ASSERT_LESS_OR_EQUAL(2304, sizeof(socket_message_Message));
    TEST_ASSERT_LESS_OR_EQUAL(2304, sizeof(socket_message_CorrelationResponse));
    TEST_ASSERT_LESS_OR_EQUAL(768, sizeof(socket_message_CorrelationRequest));
}

// The I2C scan reports every address of 0x08 to 0x77 that answers. A reply with all 112 still goes out of the static
// encode buffer, with the largest correlation id and status, rather than through the heap fallback that can fail.
void test_an_i2c_scan_reply_holds_every_address_and_fits_the_encode_buffer() {
    static socket_message_Message msg = socket_message_Message_init_zero;
    msg.which_message = socket_message_Message_correlation_response_tag;
    auto &res = msg.message.correlation_response;
    res.correlation_id = UINT32_MAX;
    res.status_code = UINT32_MAX;
    res.which_response = socket_message_CorrelationResponse_i2c_scan_data_tag;
    auto &scan = res.response.i2c_scan_data;
    TEST_ASSERT_GREATER_OR_EQUAL(112, std::size(scan.devices));
    for (int address = 0x08; address <= 0x77; address++) scan.devices[scan.devices_count++].address = address;

    size_t size = 0;
    TEST_ASSERT_TRUE(pb_get_encoded_size(&size, socket_message_Message_fields, &msg));
    TEST_ASSERT_LESS_OR_EQUAL(PROTO_BUFFER_SIZE, size);
}

template <typename Text>
void fillText(Text &field) {
    memset(field, 'x', sizeof(field) - 1);
    field[sizeof(field) - 1] = '\0';
}

// The settings and the status share these members, each with its own caps.
template <typename Mdns>
void fillToEveryCap(Mdns &mdns) {
    auto fillRecord = [](api_MDNSTxtRecord &record) {
        fillText(record.key);
        fillText(record.value);
    };
    fillText(mdns.hostname);
    fillText(mdns.instance);
    mdns.global_txt_records_count = std::size(mdns.global_txt_records);
    for (auto &record : mdns.global_txt_records) fillRecord(record);
    mdns.services_count = std::size(mdns.services);
    for (auto &service : mdns.services) {
        fillText(service.service);
        fillText(service.protocol);
        service.port = UINT32_MAX;
        service.txt_records_count = std::size(service.txt_records);
        for (auto &record : service.txt_records) fillRecord(record);
    }
}

size_t encodedSize(const socket_message_Message &msg) {
    size_t size = 0;
    TEST_ASSERT_TRUE(pb_get_encoded_size(&size, socket_message_Message_fields, &msg));
    return size;
}

// A settings push goes out of the adapter's static encode buffer; 64 bytes stay in hand for the frame around it.
void test_the_largest_mdns_settings_push_fits_the_encode_buffer() {
    static socket_message_Message msg = socket_message_Message_init_zero;
    msg.which_message = MessageTraits<api_MDNSSettings>::tag;
    fillToEveryCap(msg.message.mdns_settings);
    TEST_ASSERT_LESS_OR_EQUAL(PROTO_BUFFER_SIZE - 64, encodedSize(msg));
}

// ble.h cannot be included on the host (it needs NimBLE); it sets BLE_MAX_FRAME_SIZE to PROTO_BUFFER_SIZE, which is
// what the limit is taken from here. The update arrives in one frame, with the largest correlation id.
void test_the_largest_mdns_update_fits_one_ble_frame() {
    static socket_message_Message msg = socket_message_Message_init_zero;
    msg.which_message = socket_message_Message_correlation_request_tag;
    auto &req = msg.message.correlation_request;
    req.correlation_id = UINT32_MAX;
    req.which_request = socket_message_CorrelationRequest_mdns_settings_update_tag;
    fillToEveryCap(req.request.mdns_settings_update);
    TEST_ASSERT_LESS_OR_EQUAL(PROTO_BUFFER_SIZE - 64, encodedSize(msg));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_the_protocol_structs_stay_small);
    RUN_TEST(test_an_i2c_scan_reply_holds_every_address_and_fits_the_encode_buffer);
    RUN_TEST(test_the_largest_mdns_settings_push_fits_the_encode_buffer);
    RUN_TEST(test_the_largest_mdns_update_fits_one_ble_frame);
    return UNITY_END();
}
