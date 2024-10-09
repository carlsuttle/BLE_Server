#include <BLE_Server2.h>

Preferences preferences;

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("Starting ESP Server");

  Wire.setPins(6,7);
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  preferences.begin("prefs", false);

  setupBLE();
  setupWIFI();
  setupIMU();
  setupGUI();

  Serial.println( "Setup done" );
  
} 

void loop() {

  #ifdef PRINTMAIN
    static int i =0;
    if (i > 1000){
      Serial.print( "Iteration time = ");
      Serial.println( lastiteration);
      i = 0;
    }
    i++;
  #endif 

  doBLE();
  doWIFI();
  doIMU();
  doGUI();
  BITSerial();

}
//#endif