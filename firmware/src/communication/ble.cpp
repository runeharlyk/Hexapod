#include <communication/ble.h>
#include <esp_log.h>
#include <cstring>

static const char *TAG = "BLE";

BLE::~BLE() {
    _taskRunning = false;
    if (_server) NimBLEDevice::deinit(true);
    if (_messageQueue) vQueueDelete(_messageQueue);
    if (_txMutex) vSemaphoreDelete(_txMutex);
}

void BLE::begin() {
    _messageQueue = xQueueCreate(30, sizeof(BLEMessage));
    if (!_messageQueue) {
        ESP_LOGE(TAG, "Failed to create message queue");
        return;
    }

    _txMutex = xSemaphoreCreateMutex();

    _taskRunning = true;
    if (xTaskCreatePinnedToCore(messageProcessingTask, "BLE_Process", 8192, this, 6, &_processingTask, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create processing task");
        _taskRunning = false;
        vQueueDelete(_messageQueue);
        _messageQueue = nullptr;
        return;
    }

    setup();
}

void BLE::setup() {
    NimBLEDevice::init("Hexapod");
    NimBLEDevice::setMTU(255);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    _server = NimBLEDevice::createServer();
    _server->setCallbacks(new ServerCallbacks(this));

    NimBLEService *service = _server->createService(BLE_SERVICE_UUID);
    _txCharacteristic = service->createCharacteristic(BLE_CHARACTERISTIC_TX, NIMBLE_PROPERTY::NOTIFY);
    _rxCharacteristic =
        service->createCharacteristic(BLE_CHARACTERISTIC_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    _rxCharacteristic->setCallbacks(new RXCallbacks(this));
    service->start();

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setName("Hexapod");
    advertising->start();

    ESP_LOGI(TAG, "NUS BLE service advertising as Hexapod");
}

void BLE::ServerCallbacks::onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) {
    _service->_deviceConnected = true;
    _service->_rxBuffer.clear();
    server->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 400);
    ESP_LOGI(TAG, "Client connected: %u", connInfo.getConnHandle());
}

void BLE::ServerCallbacks::onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) {
    _service->_deviceConnected = false;
    _service->_rxBuffer.clear();
    _service->removeClient(0);
    NimBLEDevice::startAdvertising();
    ESP_LOGI(TAG, "Client disconnected (reason %d), advertising", reason);
}

void BLE::RXCallbacks::onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) {
    NimBLEAttValue value = characteristic->getValue();
    size_t len = value.length();
    if (len == 0 || len > BLE_MAX_MESSAGE_SIZE) return;

    BLEMessage msg;
    memcpy(msg.data, value.data(), len);
    msg.length = len;
    if (xQueueSend(_service->_messageQueue, &msg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Queue full, dropping frame");
    }
}

void BLE::messageProcessingTask(void *parameter) {
    BLE *self = static_cast<BLE *>(parameter);
    BLEMessage msg;
    while (self->_taskRunning) {
        if (xQueueReceive(self->_messageQueue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            self->reassemble(msg.data, msg.length);
        }
    }
    vTaskDelete(nullptr);
}

// Framing is [uint16 LE length][payload] chunked to the MTU; rebuild each complete message here.
void BLE::reassemble(const uint8_t *data, size_t len) {
    _rxBuffer.insert(_rxBuffer.end(), data, data + len);
    size_t pos = 0;
    while (_rxBuffer.size() - pos >= 2) {
        size_t msgLen = _rxBuffer[pos] | (_rxBuffer[pos + 1] << 8);
        if (_rxBuffer.size() - pos < 2 + msgLen) break;
        handleIncoming(_rxBuffer.data() + pos + 2, msgLen, 0);
        pos += 2 + msgLen;
    }
    if (pos > 0) _rxBuffer.erase(_rxBuffer.begin(), _rxBuffer.begin() + pos);
}

void BLE::send(const uint8_t *data, size_t len, int cid) {
    if (!_deviceConnected || !_txCharacteristic) return;

    static constexpr size_t CHUNK = 180;
    const uint8_t header[2] = {(uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF)};
    uint8_t buf[CHUNK];
    size_t bufLen = 0;

    auto flush = [&]() {
        if (bufLen == 0) return;
        _txCharacteristic->setValue(buf, bufLen);
        _txCharacteristic->notify();
        bufLen = 0;
    };
    auto append = [&](const uint8_t *src, size_t n) {
        for (size_t i = 0; i < n;) {
            size_t take = CHUNK - bufLen < n - i ? CHUNK - bufLen : n - i;
            memcpy(buf + bufLen, src + i, take);
            bufLen += take;
            i += take;
            if (bufLen == CHUNK) flush();
        }
    };

    if (_txMutex) xSemaphoreTake(_txMutex, portMAX_DELAY);
    append(header, 2);
    append(data, len);
    flush();
    if (_txMutex) xSemaphoreGive(_txMutex);
}
