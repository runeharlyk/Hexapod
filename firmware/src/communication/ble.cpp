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

    // advertising->start() starts the GATT server, and with it every service created above.
    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setName("Hexapod");
    advertising->start();

    ESP_LOGI(TAG, "NUS BLE service advertising as Hexapod");
}

void BLE::ServerCallbacks::onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) {
    _service->_connHandle = connInfo.getConnHandle();
    _service->_deviceConnected = true;
    _service->requestRxReset();
    server->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 400);
    ESP_LOGI(TAG, "Client connected: %u", connInfo.getConnHandle());
}

void BLE::ServerCallbacks::onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) {
    _service->_deviceConnected = false;
    _service->requestRxReset();
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

// Runs on the NimBLE host task, while _rxBuffer belongs to BLE_Process. Frames still queued from the
// old connection are discarded, and the empty marker tells BLE_Process to drop its partial frame.
void BLE::requestRxReset() {
    xQueueReset(_messageQueue);
    BLEMessage marker{};
    xQueueSendToFront(_messageQueue, &marker, 0);
}

void BLE::messageProcessingTask(void *parameter) {
    BLE *self = static_cast<BLE *>(parameter);
    BLEMessage msg;
    while (self->_taskRunning) {
        if (xQueueReceive(self->_messageQueue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (msg.length == 0) self->_rxBuffer.clear();
            else self->reassemble(msg.data, msg.length);
        }
    }
    vTaskDelete(nullptr);
}

// Framing is [uint16 LE length][payload] chunked to the MTU; rebuild each complete message here.
void BLE::reassemble(const uint8_t *data, size_t len) {
    if (_rxBuffer.size() + len > BLE_RX_BUFFER_LIMIT) {
        ESP_LOGW(TAG, "RX buffer would exceed %u bytes, dropping stream to resync", (unsigned)BLE_RX_BUFFER_LIMIT);
        _rxBuffer.clear();
        return;
    }

    _rxBuffer.insert(_rxBuffer.end(), data, data + len);
    size_t pos = 0;
    while (_rxBuffer.size() - pos >= 2) {
        size_t msgLen = _rxBuffer[pos] | (_rxBuffer[pos + 1] << 8);
        if (msgLen == 0 || msgLen > BLE_MAX_FRAME_SIZE) {
            ESP_LOGW(TAG, "Bogus frame length %u, dropping stream to resync", (unsigned)msgLen);
            _rxBuffer.clear();
            return;
        }
        if (_rxBuffer.size() - pos < 2 + msgLen) break;
        handleIncoming(_rxBuffer.data() + pos + 2, msgLen, 0);
        pos += 2 + msgLen;
    }
    if (pos > 0) _rxBuffer.erase(_rxBuffer.begin(), _rxBuffer.begin() + pos);
}

// notify() fails when NimBLE has no mbuf for the chunk after its own 10 ms of retries (BLE_HS_ENOMEM), or when the
// link is gone. ble_frame::send retries a chunk after the first and ends the link rather than leave a frame cut short.
bool BLE::send(const uint8_t *data, size_t len, int cid) {
    if (!_deviceConnected || !_txCharacteristic) return false;

    if (_txMutex) xSemaphoreTake(_txMutex, portMAX_DELAY);
    const bool sent = ble_frame::send(
        data, len, [this](const uint8_t *chunk, size_t n) { return _txCharacteristic->notify(chunk, n); },
        [] { vTaskDelay(pdMS_TO_TICKS(10)); },
        [this] {
            ESP_LOGW(TAG, "A frame's chunk found no mbuf; ending the link rather than leave the frame cut short");
            // No later frame may follow the cut one on this link; onDisconnect forgets the client.
            _deviceConnected = false;
            _server->disconnect(_connHandle);
        });
    if (_txMutex) xSemaphoreGive(_txMutex);
    return sent;
}
