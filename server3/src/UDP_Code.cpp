#include <BLE_Server2.h>

#include <WiFi.h>
#include <AsyncUDP.h>
#include "GDL90.h"

// UDP ports
const int UdpGDLPort = 4000;

//network credentials
const char *gdl_ssid = "GenericAP";
const char *gdl_password = "password";

AsyncUDP udp;
GDL90 Gdl90;

bool wifiConnected;
unsigned long udp_previousMillis = 0;
unsigned long udp_currentMillis = 0;
const long udp_interval = 500;  // Interval at which to check Wi-Fi status (milliseconds)

//gdl variables
float gdlheading;
float gdlpitch;
float gdlroll;
bool  gdl_valid;

void connectToWiFi() {
  WiFi.begin(gdl_ssid, gdl_password);
  Serial.println("Attempting to connect to Wi-Fi...");
}

void setupWIFI() {

// Start Wi-Fi connection
connectToWiFi();

if (udp.listen(WiFi.localIP(), UdpGDLPort)) {
    udp.onPacket([](AsyncUDPPacket packet) {
      int length = packet.length();
      uint8_t packetBuffer[length];
      packet.readBytes(packetBuffer, length);
      if(packetBuffer[1]==0x65 && packetBuffer[2]==0x01) {     //AHRS message type
        //if message is invalid decodeAHRSMessage doesn't change heading,pitch,roll which were passed into the method
        gdl_valid = Gdl90.decodeAHRSMessage(&gdlheading, &gdlpitch, &gdlroll, packetBuffer, &length);
        #ifdef PRINTUDP
          if(gdl_valid){
            Serial.print("GDL_Pitch ");
            Serial.print(gdlpitch);
            Serial.print(", GDL_Roll ");
            Serial.println(gdlroll+5);
          }
        #endif
       }
    });
  }
}

void doWIFI() {
  unsigned long udp_currentMillis = millis();

  // Check Wi-Fi connection status at regular intervals
  if (udp_currentMillis - udp_previousMillis >= udp_interval) {
    udp_previousMillis = udp_currentMillis;
    
    if (WiFi.status() == WL_CONNECTED) {
      if (!wifiConnected) {
        wifiConnected = true;
        Serial.println("\nConnected to Wi-Fi");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
      }
    } else {
      if (wifiConnected) {
        wifiConnected = false;
        Serial.println("Disconnected from Wi-Fi");
        connectToWiFi();  // Attempt to reconnect
      }
    }
  }
  // Other non-blocking tasks can go here
}

//#endif