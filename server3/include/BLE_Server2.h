#ifndef _BLE_Server2_
#define _BLE_Server2_

#include <Arduino.h>
#include <Wire.h>
#include "lvgl.h"
#include <Preferences.h>

// Declare variables as extern
extern String inputString;  // A string to hold incoming characters
extern Preferences preferences;

// global variables accessed by the GUI
//**********************************************
// g meter
extern float zacceleration;
extern float imuroll;
extern float imupitch;
extern float imuyaw;

// Screen and GUI Rotation
#define GUIROTATION 1 // 0 = usb to bottom, 1 = usb to the right
#define DIFFLIMIT 2   // gui gyro and accelerometer difference indicator red above this value
extern int aiDisplaysource;
extern unsigned long lastiteration;
extern int i;

// GPS variables
// Defines below describe the contents of the ble_floatValues array
#define AOA 0
#define LAT 1
#define LON 2
#define ALT 3
#define CRS 4
#define SPEEDMS 5
#define TIME 6
#define YAWRATE 7
#define SPEED 8
#define ACCELERATION 9

extern double speed, course;
extern double rot_deg_min;
extern float  ble_floatValues[10]; 
extern double accelXGPS;
extern double centripetalAccel;
extern double deltaYaw;
extern double deltaSpeed;
extern double radiusofturn;
extern double yawRate;
extern double acceleration;
extern double dt;

// Time: Variables to store the synchronized time
extern float timeValues[3];        // Array to hold the received time values (hours, minutes, seconds)
extern float syncHours, syncMinutes, syncSeconds;
extern unsigned long lastSyncMillis;
extern unsigned int gmtoffset;

// External attitude
extern bool wifiConnected;
extern float gdlheading;
extern float gdlpitch;
extern float gdlroll;
extern bool gdl_valid;

// IMU globals so they can be displayed in Serial BIT
extern double accXangle, accYangle, accZangle; // Angle calculate using the accelerometer only
extern double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
extern double kgyroXangle, kgyroYangle, kgyroZangle; // Angle calculate using the gyro only
extern double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
extern double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter
extern double magAngleX, magAngleY, magAngleZ;
extern float compfiltervariable;
extern float pitchdifference;
extern float rolldifference;

//#define PRINTBLE
//#define PRINTUDP
//#define PRINTIMU
//#define PRINTGUI
//#define PRINTMAIN

#define VERSION 0.3

// Function declarations
void setupBLE();
void setupWIFI();
void setupIMU();
void setupGUI();

void doBLE();
void doWIFI();
void doIMU();
void doGUI();
void BITSerial();

void IMUCalibrate();

#endif
