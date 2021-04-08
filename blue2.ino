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


#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "battery.h"
#include "camera_pins.h"
#include "led.h"
#include "bmm8563.h"

#include "esp_camera.h"
#include "src/base64.hpp"







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

void led_breathe_test() {
  for (int16_t i = 0; i < 1024; i++) {
    led_brightness(i);
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  for (int16_t i = 1023; i >= 0; i--) {
    led_brightness(i);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}


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

  // LED
  led_init(CAMERA_LED_GPIO);
  led_breathe_test();

  // camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;//YUV422,GRAYSCALE,RGB565,JPEG
  config.frame_size = FRAMESIZE_FHD;
  config.jpeg_quality = 10;//0-63 lower number means higher quality
  config.fb_count = 2;//if more than one, i2s runs in continuous mode. Use only with JPEG

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  s->set_vflip(s, 1);//flip it back
  s->set_brightness(s, 1);//up the blightness just a bit
  s->set_saturation(s, -2);//lower the saturation

  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_FHD);

  Serial.print("Camera Ready!");


}

void loop()
{

    camera_fb_t *fb = esp_camera_fb_get();
    if ( fb ) {
      // Serial.printf("width: %d, height: %d, buf: 0x%x, len: %d\n", fb->width, fb->height, fb->buf, fb->len);
      unsigned int base64_length = encode_base64_length(fb->len);
      unsigned char *base64buff = new unsigned char[base64_length+1];
      base64buff[base64_length] = '\0';
      encode_base64(fb->buf, fb->len, base64buff);
      // char *serverName = "http://192.168.0.16:1880/image";
      // HTTPClient http;
      // http.begin(serverName);
      // http.addHeader("Content-Type", "text/plain");
      // int httpResponseCode = http.POST(reinterpret_cast<char*>(base64buff));
      // Serial.printf("%d", httpResponseCode);
      Serial.printf("%s", base64buff);
      delete [] base64buff;
      esp_camera_fb_return(fb);
    }

    if (deviceConnected) {

      // Serial.print("-1: ");
      // Serial.println(digitalRead(-1));

      portENTER_CRITICAL_ISR(&storeDataMux);
      // Serial.println("connected");
      if (bleDataIsReceived) {
        led_breathe_test();
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

  delay(5000);
}
