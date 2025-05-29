#include <bluetooth_service.h>
#include <PsychicHttp.h>

BluetoothService::BluetoothService()
    : StatefulService<BluetoothSettings>(),
      endpoint(BluetoothSettings::read, BluetoothSettings::update, this),
      _persistence(BluetoothSettings::read, BluetoothSettings::update, this, BLUETOOTH_SETTINGS_FILE),
      _server(nullptr),
      _txCharacteristic(nullptr),
      _rxCharacteristic(nullptr),
      _deviceConnected(false),
      _cmdSubHandle(nullptr),
      _tempSubHandle(nullptr) {
    addUpdateHandler([&](const String& originId) { restart(); }, false);
}

BluetoothService::~BluetoothService() {
    EventBus::unsubscribe<Command>(_cmdSubHandle);
    EventBus::unsubscribe<Temp>(_tempSubHandle);
    if (_server) {
        BLEDevice::deinit(true);
    }
}

void BluetoothService::begin() {
    _persistence.readFromFS();

    _cmdSubHandle = EventBus::subscribe<Command>([this](Command const& c) {
        if (_deviceConnected) emit(COMMAND, c);
    });

    _tempSubHandle = EventBus::subscribe<Temp>([this](Temp const& t) {
        if (_deviceConnected) emit(TEMP, t);
    });

    setup();
}

void BluetoothService::setup() {
    const std::string& deviceName = state().settings.deviceName.c_str();
    ESP_LOGI("BluetoothService", "Initializing BLE with device name: %s", deviceName.c_str());

    BLEDevice::init(deviceName);
    _server = BLEDevice::createServer();
    _server->setCallbacks(new ServerCallbacks(this));

    BLEService* service = _server->createService(SERVICE_UUID);

    _txCharacteristic = service->createCharacteristic(CHARACTERISTIC_TX, BLECharacteristic::PROPERTY_NOTIFY);
    _txCharacteristic->addDescriptor(new BLE2902());

    _rxCharacteristic = service->createCharacteristic(CHARACTERISTIC_RX, BLECharacteristic::PROPERTY_WRITE);
    _rxCharacteristic->setCallbacks(new RXCallbacks(this));

    service->start();
    _server->getAdvertising()->start();

    ESP_LOGI("BluetoothService", "BLE UART service started, advertising as %s", deviceName.c_str());
}

void BluetoothService::restart() {
    ESP_LOGI("BluetoothService", "Restarting BLE service due to settings update.");
    if (_server) {
        BLEDevice::deinit(true);
        _server = nullptr;
        _txCharacteristic = nullptr;
        _rxCharacteristic = nullptr;
    }
    setup();
}

void BluetoothService::ServerCallbacks::onConnect(BLEServer* pServer) {
    _service->_deviceConnected = true;
    ESP_LOGI("BluetoothService", "Client connected");
}

void BluetoothService::ServerCallbacks::onDisconnect(BLEServer* pServer) {
    _service->_deviceConnected = false;
    ESP_LOGI("BluetoothService", "Client disconnected");
    pServer->startAdvertising();
    ESP_LOGI("BluetoothService", "Restarting advertising");
}

void BluetoothService::RXCallbacks::onWrite(BLECharacteristic* characteristic) {
    std::string value = characteristic->getValue();
    if (!value.empty()) {
        _service->handleReceivedData(value);
    }
}

void BluetoothService::handleReceivedData(const std::string& data) {
    // CALLS_PER_SECOND(new_message);

    JsonDocument doc;
#if USE_MSGPACK
    DeserializationError error = deserializeMsgPack(doc, data);
#else
    DeserializationError error = deserializeJson(doc, data);
#endif
    if (error) {
        throw std::runtime_error(error.c_str());
    }

    serializeJson(doc, Serial);
    Serial.println();
    JsonArray obj = doc.as<JsonArray>();

    message_type_t type = obj[0].as<message_type_t>();

    int cid = 0;

    switch (type) {
        case CONNECT: {
            message_topic_t topic = obj[1].as<message_topic_t>();
            ESP_LOGI("BluetoothService", "Connecting to topic: %d", topic);
            subscribe(topic, cid);
            break;
        }
        case DISCONNECT: {
            message_topic_t topic = obj[1].as<message_topic_t>();
            ESP_LOGI("BluetoothService", "Disconnecting to topic: %d", topic);
            unsubscribe(topic, cid);
            break;
        }

        case EVENT: {
            message_topic_t topic = obj[1].as<message_topic_t>();
            if (topic == TEMP) {
                Temp payload;
                payload.fromJson(obj[2]);
                EventBus::publish<Temp>(payload, _tempSubHandle);
            } else if (topic == COMMAND) {
                Command payload;
                payload.fromJson(obj[2]);
                EventBus::publish<Command>(payload, _tempSubHandle);
            } else if (topic == MODE) {
                Mode payload;
                payload.fromJson(obj[2]);
                EventBus::publish<Mode>(payload, _tempSubHandle);
            };

            ESP_LOGD("BluetoothService", "Got payload for topic: %d", topic);
            break;
        }

        default: ESP_LOGW("BluetoothService", "Unknown message type: %d", type); break;
    }
}

void BluetoothService::send(const char* data) {
    if (_deviceConnected) {
        _txCharacteristic->setValue((uint8_t*)data, strlen(data));
        _txCharacteristic->notify();
    }
}

void BluetoothService::sendData(const std::string& data) {
    if (_deviceConnected && _txCharacteristic) {
        _txCharacteristic->setValue(data);
        _txCharacteristic->notify();
        ESP_LOGI("BluetoothService", "Sent: %s", data.c_str());
    } else {
        ESP_LOGW("BluetoothService", "Cannot send data, no device connected or TX characteristic invalid.");
    }
}

void BluetoothService::sendData(uint8_t* data, size_t length) {
    if (_deviceConnected && _txCharacteristic) {
        _txCharacteristic->setValue(data, length);
        _txCharacteristic->notify();
        ESP_LOGI("BluetoothService", "Sent %d bytes", length);
    } else {
        ESP_LOGW("BluetoothService", "Cannot send data, no device connected or TX characteristic invalid.");
    }
}

esp_err_t BluetoothService::getStatus(PsychicRequest* request) {
    PsychicJsonResponse response = PsychicJsonResponse(request, false);
    JsonObject root = response.getRoot();
    return response.send();
}
