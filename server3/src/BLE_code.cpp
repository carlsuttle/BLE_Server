// BLE libraries

#include <BLE_Server2.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// BLE Global Variables
// The UUIDs for the service and characteristics
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // For float data
#define TIME_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // For time data (hours, minutes, seconds)

float ble_floatValues[10];  // Array to hold the received float values
float timeValues[3];        // Array to hold the received time values (hours, minutes, seconds)
float syncHours, syncMinutes, syncSeconds;
unsigned long lastSyncMillis;
unsigned int gmtoffset;

// BLE timeout for detecting if the client has stopped sending data (in milliseconds)
unsigned long ble_timeout = 10000;
unsigned long ble_lastUpdate = 0;

// Callback for BLE Server
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Client connected");
      ble_lastUpdate = millis();  // Reset the ble_timeout when a client connects
      BLEDevice::getAdvertising()->stop();  // Stop advertising once a client connects
    }

    void onDisconnect(BLEServer* pServer) {
      Serial.println("Client disconnected");
      BLEDevice::startAdvertising();  // Restart advertising when the client disconnects
    }
};

// Callback for BLE Characteristic
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    // Get the characteristic value as a byte array
    std::string value = pCharacteristic->getValue();
    
    if (pCharacteristic->getUUID().toString() == CHARACTERISTIC_UUID) {
      // Handle receiving float data (ble_floatValues)
      if (value.length() == sizeof(ble_floatValues)) {
        memcpy(ble_floatValues, value.c_str(), sizeof(ble_floatValues));

        // Debug: Print the received floats
        #ifdef PRINTBLE
        Serial.print("Received floats: ");
        for (int i = 0; i < 10; i++) {
          Serial.print(ble_floatValues[i]);
          if (i < 9) Serial.print(", ");
        }
        Serial.println();
        #endif

        // Reset the ble_timeout when a new value is received
        ble_lastUpdate = millis();
      } else {
        Serial.println("Error: Received data size does not match expected float array size.");
      }
    } 
    else if (pCharacteristic->getUUID().toString() == TIME_CHAR_UUID) {
      // Handle receiving time data (timeValues)
      if (value.length() == sizeof(timeValues)) {
        memcpy(timeValues, value.c_str(), sizeof(timeValues));
        syncHours = timeValues[0];
        syncMinutes = timeValues[1];
        syncSeconds = timeValues[2];
        lastSyncMillis = millis();
        
        // Debug: Print the received time data
        #ifdef PRINTBLE
          Serial.print("Received Time - Hours: ");
          Serial.print(timeValues[0]);
          Serial.print(", Minutes: ");
          Serial.print(timeValues[1]);
          Serial.print(", Seconds: ");
          Serial.println(timeValues[2]);
        #endif

        // Reset the ble_timeout when time data is received
        ble_lastUpdate = millis();
      } else {
        Serial.println("Error: Received time data size does not match expected size.");
      }
    }
  }
};

void setupBLE() {
  // Initialize BLE
  Serial.println("Starting BLE AOA_Server");
  BLEDevice::init("AOA_Server");

  // Create BLE server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE characteristic for float data
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID,
                                        BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_NOTIFY
                                      );

  // Set callback for float characteristic
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  // Create BLE characteristic for time data
  BLECharacteristic *pTimeCharacteristic = pService->createCharacteristic(
                                           TIME_CHAR_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE
                                         );

  // Set callback for time characteristic
  pTimeCharacteristic->setCallbacks(new MyCallbacks());
  pTimeCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Helps with iPhone connection issues
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Characteristic defined");
}

void doBLE() {
  // Check if the BLE timeout has been reached
  if (millis() - ble_lastUpdate > ble_timeout) {
    Serial.println("BLE Client has stopped sending data");
    ble_lastUpdate = millis();  // Reset the ble_timeout to avoid continuous printing
  }
}

//#endif
