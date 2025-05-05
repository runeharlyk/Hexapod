#include <spot.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

DRAM_ATTR Spot spot;

BLEServer *server;
BLECharacteristic *txCharacteristic;
bool deviceConnected = false;


#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        deviceConnected = true;
        Serial.println("Client connected");
    }
    void onDisconnect(BLEServer *pServer) {
        deviceConnected = false;
        Serial.println("Client disconnected");
    }
};

class RXCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) {
        std::string value = characteristic->getValue();
        if (!value.empty()) {
            Serial.print("Received: ");
            Serial.println(value.c_str());

            std::string response = "ACK: " + value;
            txCharacteristic->setValue(response);
            txCharacteristic->notify();

            Serial.print("Sent: ");
            Serial.println(response.c_str());
        }
    }
};

void IRAM_ATTR SpotControlLoopEntry(void *) {
    ESP_LOGI("main", "Setup complete now running tsk");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;
    for (;;) {
        // spot.readSensors();
        spot.planMotion();
        spot.updateActuators();
        // spot.emitTelemetry();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void setup() {
    Serial.begin(115200);

    spot.initialize();

    g_taskManager.createTask(SpotControlLoopEntry, "Control task", 4096, nullptr, 5);

    BLEDevice::init("Hexapod");
    server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    BLEService *service = server->createService(SERVICE_UUID);

    txCharacteristic = service->createCharacteristic(CHARACTERISTIC_TX, BLECharacteristic::PROPERTY_NOTIFY);
    txCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *rxCharacteristic =
        service->createCharacteristic(CHARACTERISTIC_RX, BLECharacteristic::PROPERTY_WRITE);
    rxCharacteristic->setCallbacks(new RXCallbacks());

    service->start();
    server->getAdvertising()->start();

    Serial.println("BLE UART started, waiting for client...");
}

void loop() { vTaskDelete(nullptr); }
