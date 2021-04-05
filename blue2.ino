// https://github.com/bigw00d/esp32_arduino_ble/edit/master/examples/simple_ble_echo_back/simple_ble_echo_back.ino

//  note:
//    use Arduino 1.8.5
//    Arduino board manager
//      https://dl.espressif.com/dl/package_esp32_index.json
//    ボード ESP32 Dev Module
//    Flash Mode  QIO
//    Flash Frequency 80MHz
//    Flash Size  4M (32Mb)
//    Partition Scheme  No OTA (2MB APP/2MB FATFS) <- important!
//    Upload Speed  115200
//    Core Debug Level なし

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <WiFiMulti.h>

#define SERVICE_UUID           "00001141-0000-1000-8000-00805f9b34fb" // UART service UUID
#define CHARACTERISTIC_UUID_RX "00001142-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_TX "00001143-0000-1000-8000-00805f9b34fb"

// const char* uuidOfService = "00001101-0000-1000-8000-00805f9b34fb";
// const char* uuidOfRxChar = "00001142-0000-1000-8000-00805f9b34fb";
// const char* uuidOfTxChar = "00001143-0000-1000-8000-00805f9b34fb";


// #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
// #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
// #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

boolean bleDataIsReceived;
std::string storedValue;
portMUX_TYPE storeDataMux = portMUX_INITIALIZER_UNLOCKED;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      Serial.println("write in esp32");

      if (rxValue.length() > 0) {
        portENTER_CRITICAL_ISR(&storeDataMux);
        storedValue = rxValue;
        bleDataIsReceived = true;
        portEXIT_CRITICAL_ISR(&storeDataMux);
      }
    }

    void onRead(BLECharacteristic *pCharacteristic) {
      Serial.println("read in esp32");
      pCharacteristic->setValue("Hello from esp32! onRead");
    }

  // void onWrite(BLECharacteristic *pCharacteristic) {
  //   Serial.println("write");
  //   std::string value = pCharacteristic->getValue();
  //   storedValue = value;
  //   bleDataIsReceived = true;
  //   Serial.println(value.c_str());
  // }


};


void setup()
{
  Serial.begin(115200); // Debug Print
  WiFi.disconnect(true); //disable wifi

  bleDataIsReceived = false;

  // Create the BLE Device
  BLEDevice::init("timerx");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

}

void loop()
{
    if (deviceConnected) {
      portENTER_CRITICAL_ISR(&storeDataMux);
      // Serial.println("connected");
      if (bleDataIsReceived) {
        bleDataIsReceived = false;
        Serial.println("received string:");
        Serial.println(storedValue.c_str());
        pTxCharacteristic->setValue(storedValue);
        pTxCharacteristic->notify();
      }
      portEXIT_CRITICAL_ISR(&storeDataMux);
      delay(10); // bluetooth stack will go into congestion, if too many packets are sent
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }

    // connecting
    if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }  

}

