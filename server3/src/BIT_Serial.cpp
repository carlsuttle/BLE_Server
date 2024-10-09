
#include <BLE_Server2.h>

unsigned long bitstartMillis;  
unsigned long bitcurrentMillis;
const unsigned long bitperiod = 30;  //the value is a number of milliseconds
String inputString;  // A string to hold incoming characters
float compfiltervariable;
int  aiDisplaysource;
double speed, course, rot_deg_min, dt;
double accXangle, accYangle, accZangle; // Angle calculate using the gyro only
double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double kgyroXangle, kgyroYangle, kgyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter
double magAngleX, magAngleY, magAngleZ;
double accelXGPS;
double centripetalAccel;
bool stringComplete = false;  // Whether the string is complete


void BITSerial() {

  static int outputlog = 0;
  static int lastXPconnection = 0;
  unsigned int bytecount = 0;
  char buff[1024];
  char incomingByte;

  unsigned long  bitcurrentMillis = millis();
  if (bitcurrentMillis - bitstartMillis < bitperiod) return; //test whether the period has elapsed
    
  bitstartMillis = bitcurrentMillis;

  while (Serial.available() && bytecount < 10) {
    incomingByte = Serial.read();

    bytecount++;
    if (incomingByte == 'h') {
      // sprintf(buff, "Version No %s", VERSION);
      // Serial.println(buff);
      sprintf(buff, "Welcome to CirrusWatch \n%s\nCompiled %s %s\n", __FILE__, __DATE__, __TIME__);
      Serial.println(buff);

      //MENU
      sprintf(buff, 
      "Type 'a' to select output angle log.\n"
      "Type 'c' to change complimentary filter control variable.\n"
      "Type 'd' to change attitude display source.  Raw, Complimentary, Kalman, Madgewick\n"
      "Type 'g' to see gps data.\n"
      "Type 'i' to see derived gps speeds and accelerations.\n"
      "Type 'f' to see gps derived aircraft parameters.\n"
      // "Type 'r' to toggle roll sensor and servo polarity.\n"
      // "Type 's' to see switch states.\n"
      // "Type 't' to start/stop dynamic test.\n"
      // "Type 'v' to see X-Plane variables.\n"
      // "Type 'w' to test watchdog hold the UP, PTT and DISCO buttons on for 10 sec.\n"
      // "Type 'z' to set roll offset.\n"
      // "Type 'C' to test crash report **DISCONNECT ROLL SERVO FOR THIS TEST**.\n"
      "Type 'x' to stop logging.\n");
      Serial.println(buff);
      outputlog = 0;
    }

  
    if (incomingByte == 'x') outputlog = 0;
    if (incomingByte == 'a') outputlog = 1;
    if (incomingByte == 'c') outputlog = 2;
    if (incomingByte == 'd') outputlog = 3;
    if (incomingByte == 'g') outputlog = 4;
    if (incomingByte == 'i') outputlog = 5;
    if (incomingByte == 'f') outputlog = 6;
    // if (incomingByte == 'v') outputlog = 8;
    // if (incomingByte == 'f') outputlog = 9;
    // if (incomingByte == 'a') outputlog = 10;
    // if (incomingByte == 'm') outputlog = 11;
    // if (incomingByte == 'n') outputlog = 12;
    // if (incomingByte == 'r') outputlog = 13;

    if (outputlog == 102) {
      if (incomingByte == '\r') {
        stringComplete = true;
        // Try to convert the string to a float
        float receivedNumber = inputString.toFloat();

        // Check if the number is between 0 and 1
        if (receivedNumber >= 0.0 && receivedNumber <= 1.0) {
          Serial.print("complimentary filter control variable to  ");
          Serial.println(receivedNumber, 6);  // Print the number with 6 decimal places
          compfiltervariable = receivedNumber;
        } else {
          Serial.println("Invalid number. Please enter a number between 0.0 and 1.0");
        }
        // Clear the input string for the next number
        inputString = "";
        stringComplete = false;
      } else {
        // Append the character to the input string
        inputString += incomingByte;
      }
    }
     if (outputlog == 103) {
      if (incomingByte == '\n') {
        stringComplete = true;
        // Try to convert the string to a float
        int receivedNumber = inputString.toInt();

        // Check if the number is between 0 and 1
        if (receivedNumber >= 1 && receivedNumber <= 4) {
          Serial.print("Attitude Indicator source is   ");
          Serial.print(receivedNumber, 6);  // Print the number with 6 decimal places
          switch (receivedNumber) {
            case 1:
              Serial.println(" Raw");
            break;
            case 2:
              Serial.println(" Complimentary");
            break;
            case 3:
              Serial.println(" Kalman");
            break;
            case 4:
              Serial.println(" Madgewick");
            break;
          }
          aiDisplaysource = receivedNumber;
        } else {
          Serial.println("Invalid number. Please enter a number between 1 and 4");
        }
        // Clear the input string for the next number
        inputString = "";
        stringComplete = false;
      } else {
        // Append the character to the input string
        inputString += incomingByte;
      }
    }
  }

  //logging
  if (outputlog == 1) {
      Serial.print("dt :");
      Serial.print(dt);
      Serial.print(",");
      Serial.print("speed :");
      Serial.print(speed);
      Serial.print(",");
      Serial.print("course :");
      Serial.print(course);
      Serial.print(",");

      switch (aiDisplaysource) {
        case 1:
          Serial.print("gyroXangle :");
          Serial.print(gyroXangle);
          Serial.print(",");
          Serial.print(" gyroYangle :");
          Serial.print(gyroYangle);
          Serial.print(",");
          Serial.print(" gyroZangle :");
          Serial.print(gyroZangle);
          Serial.print(", accXangle :");
          Serial.print(accXangle);
          Serial.print(",");
          Serial.print(" accYangle :");
          Serial.print(accYangle);
          Serial.print(",");
          Serial.print(" accZangle :");
          Serial.println(accZangle);
          break;
        case 2:
          Serial.print("compAngleX :");
          Serial.print(compAngleX);
          Serial.print(",");
          Serial.print("compAngleY :");
          Serial.print(compAngleY);
          Serial.print(",");
          Serial.print(" compAngleZ :");
          Serial.println(compAngleZ);
          break;
        case 3:
          Serial.print("kalAngleX :");
          Serial.print(kalAngleX);
          Serial.print(",");
          Serial.print("kalAngleY :");
          Serial.print(kalAngleY);
          Serial.print(",");
          Serial.print(" kalAngleZ :");
          Serial.println(kalAngleZ);
          break;
        case 4:
          Serial.print("magAngleX :");
          Serial.print( magAngleX);
          Serial.print(",");
          Serial.print("  magAngleY :");
          Serial.print(magAngleY);
          Serial.print(",");
          Serial.print(" magAngleZ :");
          Serial.println( magAngleZ);
          break;
      }
  }

 if (outputlog == 2) {
  sprintf(buff, "Enter a floating point no between 0 and 1, 1.0= full gyro, 0.0 = full accelerometers");
  Serial.println(buff);
  outputlog = 102;
 }

if (outputlog == 3) {
  sprintf(buff, "Enter 1 for raw data, 2 for complimentary Filter and 3 for Kalman filtering, and 4 for Madgewick.");
  Serial.println(buff);
  outputlog = 103;
 }

 if (outputlog == 4) {
  static uint32_t lasttime = 0;
  uint32_t time = ble_floatValues[TIME];
  if (lasttime != time){
    sprintf(buff, "Lat = %.4f Lon = %.4f  Ht = %.1f Speed = %.2f Course = %.2f Yaw Rate %.4f Speed %.1f Accel %.2f Time Stamp %i", ble_floatValues[LAT], ble_floatValues[LON], ble_floatValues[ALT], ble_floatValues[SPEEDMS], ble_floatValues[CRS], ble_floatValues[YAWRATE], ble_floatValues[SPEEDMS], ble_floatValues[ACCELERATION], time );
    Serial.println(buff);
  }
  lasttime = time;
 }

  if (outputlog == 5) {
  sprintf(buff, "X_Accel = %.4f GPSAccel = %.4f  X_Accel-GPSAccel = %.4f Y_Accel= %.4f CPAcell = %.4f Y_Accel - CPAccel %.4f" , accXangle, accelXGPS, (accXangle -accelXGPS), accYangle, centripetalAccel, (accYangle - centripetalAccel));
  Serial.println(buff);
 }

  if (outputlog == 6) {
    sprintf(buff, "speed = %.1f course = %.1f  yawRate = %.4f Turn Rate = %.3f Turn Radius = %.1f CPAcceleration %.4f " , speed, course, yawRate, rot_deg_min, radiusofturn, centripetalAccel);
    Serial.println(buff);
  }
}


//#endif //BLE_Server2
