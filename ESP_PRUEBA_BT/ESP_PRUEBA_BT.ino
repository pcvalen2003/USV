
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "RF24.h"
#include <WiFi.h>
#define CE 2
#define CSN 15
#define ADC_x 39
#define ADC_y 34

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

const int ledPin = 5; // Use the appropriate GPIO pin for your setup

//Prueba de mensaje de 2 floats
//float msj1 = 0;
//float msj2 = 0;
//char buffer[32];
//const int Joystick1 = 36;
//const int Joystick2 = 39;


//BLUETOOTH//
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic) override {
    std::string rx = pLedCharacteristic->getValue();
    if (rx.size() > 0) {
      int receivedValue = static_cast<uint8_t>(rx[0]);  // Usa uint8_t para bytes

      Serial.print("Received raw byte: ");
      Serial.println(receivedValue);

      if (receivedValue == 1) {
        digitalWrite(ledPin, HIGH);
      } else {
        digitalWrite(ledPin, LOW);
      }
    } else {
      Serial.println("Received empty value");
    }
  }
};
////RADIO///
RF24 radio(CE, CSN);

typedef struct {
  uint16_t x;
  uint16_t y;
} mensaje_t;

const uint8_t direccion[5] = {0xE1, 0xF0, 0xF0, 0xE8, 0xE8};
mensaje_t mensaje;


void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(ADC_x, INPUT);
  pinMode(ADC_y, INPUT);


  if (nrf24_check == 0) {
    Serial.println("Error radio");
    while (1) { /* stall */ }
  } else Serial.println("Radio iniciada");
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(direccion);
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_LOW);
  
  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
                      SENSOR_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
                      LED_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  // notify changed value
  if (deviceConnected) {
     msj1=analogRead(Joystick1);
     msj2=analogRead(Joystick2);
     sprintf(buffer, "%.2f,%.2f", msj1, msj2);
     pSensorCharacteristic->setValue(buffer);
    //pSensorCharacteristic->setValue(String(value).c_str());
    pSensorCharacteristic->notify();
    value++;
    Serial.print("New value notified: ");
    Serial.println(value);
    delay(1000); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}
