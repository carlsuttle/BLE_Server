#include "FastIMU.h"
#include "Madgwick.h"
#include <Wire.h>

#define IMU_ADDRESS 0x6B    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
QMI8658  IMU;               //Change to the name of any supported IMU!

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

calData calib = { 0 };  //Calibration data
AccelData IMUAccel;    //Sensor data
GyroData IMUGyro;
MagData IMUMag;
Madgwick filter;

void setup() {

  Wire.setPins(6,7);
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  Serial.begin(115200);
  delay(5000);

  calib.accelBias[0] = 0.04;
  calib.accelBias[1] = 0.11;
  calib.accelBias[2] = 0.02;
  calib.gyroBias[0] = -0.26;
  calib.gyroBias[1] = 2.09;
  calib.gyroBias[2] = -0.13;

  IMU.setIMUGeometry(4);  //vertical, 0 = horezontal
  Serial.print("IMUGeometryIndex = ");
  Serial.println(4);

  int err = IMU.init(calib, IMU_ADDRESS);
  if(err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU Calibrated Quaternion example");
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }
  else {
    delay(1000);
  }
  Serial.println("Keep IMU level.");

  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
    IMU.init(calib, IMU_ADDRESS);
  
#endif

  err = IMU.setGyroRange(512);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  err = IMU.setAccelRange(4);       //THESE TWO SET THE GYRO RANGE TO ±128 DPS AND THE ACCELEROMETER RANGE TO ±4g

  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  filter.begin(0.2f);
}

void loop() {
  IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);

  Serial.print(IMUAccel.accelX*2);
  Serial.print("\t");
  Serial.print(IMUAccel.accelY*2);
  Serial.print("\t");
  Serial.println(IMUAccel.accelZ*2);
  

  // Serial.print(IMUGyro.gyroX);
  // Serial.print("\t");
  // Serial.print(IMUGyro.gyroY);
  // Serial.print("\t");
  // Serial.println(IMUGyro.gyroZ);

  if (IMU.hasMagnetometer()) {
    IMU.getMag(&IMUMag);
    filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUMag.magX, IMUMag.magY, IMUMag.magZ);
  }
  else {
    filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
  }
  Serial.print("QW: ");
  Serial.print(filter.getQuatW());
  Serial.print("\tQX: ");
  Serial.print(filter.getQuatX());
  Serial.print("\tQY: ");
  Serial.print(filter.getQuatY());
  Serial.print("\tQZ: ");
  Serial.println(filter.getQuatZ());


  float roll  = atan2((filter.getQuatW() * filter.getQuatX() + filter.getQuatY() * filter.getQuatZ()), 0.5 - (filter.getQuatX() * filter.getQuatX() + filter.getQuatY() * filter.getQuatY()));
  float pitch = asin(2.0 * (filter.getQuatW() * filter.getQuatY() - filter.getQuatX()* filter.getQuatZ()));
  float yaw   = atan2((filter.getQuatX() * filter.getQuatY() + filter.getQuatW() * filter.getQuatZ()), 0.5 - (filter.getQuatY() * filter.getQuatY() + filter.getQuatZ() * filter.getQuatZ()));
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;
  yaw *= 180.0  / PI;

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("\tPitch: ");
  Serial.print(pitch);
  Serial.print("\tYaw: ");
  Serial.println(yaw);

  delay(100);
}
