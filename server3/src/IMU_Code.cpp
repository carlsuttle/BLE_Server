#include <BLE_Server2.h>

#include <FastIMU.h>
#include <Madgwick.h>
#include <Kalman.h>

#define IMU_ADDRESS 0x6B    //Change to the address of the IMU
//#define PERFORM_CALIBRATION //Comment to disable startup calibration
#define LOWERSPEEDLIMIT 1.0
#define MAGEWICKFILTER 0.03


#define X 0
#define Y 1
#define Z 2

QMI8658  IMU;               //Change to the name of any supported IMU!
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

calData calib = { 0 };  //Calibration data
AccelData IMUAccel;    //Sensor data
GyroData IMUGyro;
Madgwick filter;

double gpsYawPrevious = 0.0;  // Previous GPS yaw (course)
double gpsYawCurrent = 0.0;   // Current GPS yaw (course

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

// double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
// double compAngleX, compAngleY, compZangle; // Calculated angle using a complementary filter
// double kalAngleX, kalAngleY, kalZangle; // Calculated angle using a Kalman filter

float zacceleration;
float imuroll;
float imupitch;
float imuyaw;
float pitchdifference;
double radiusofturn;
double yawRate;

uint32_t timer;

unsigned long imu_previousMillis = 0;
const long imu_interval = 30;  // Interval at which to check imustatus (milliseconds)
unsigned long imu_currentMillis = 0;

// *************************************DCM matrix variables
// static double DCM_Matrix[3][3];
// static double Omega_Vector[3] = {0, 0, 0};  // Gyroscope rates
// static double Update_Matrix[3][3];  // Matrix to store the updated orientation
// static double Temporary_Matrix[3][3];
static double Accel_Vector[3] = {0, 0, 0};  // Accelerometer data
static double Gyro_Vector[3] = {0, 0, 0};   // Gyroscope data

// // Low-pass filter variables
static double prevAccel[3] = {0, 0, 0};
static double prevGyro[3] = {0, 0, 0};
double alpha = 0.5;  // Smoothing factor for the low-pass filter

// // Time variables
// unsigned long prevTime;
// float dt = 0.01;  // Time step in seconds

// unsigned long startupDelay = 10000;  // 3 seconds delay
// unsigned long startupTime = millis();

//*************************************End of DCM Matrix variables
void setupIMU(){

  int err = 0;  

  IMU.setIMUGeometry(GUIROTATION + 4);  //4 = vertical, 0 = horezontal
  Serial.print("IMUGeometryIndex = ");
  Serial.println("Orientation, USB on RHS");

  if (!preferences.isKey("accelbias0")){
    IMUCalibrate();
  }else{
    calib.accelBias[0] = preferences.getFloat("accelbias0");
    calib.accelBias[1] = preferences.getFloat("accelbias1");
    calib.accelBias[2] = preferences.getFloat("accelbias2");
    calib.gyroBias[0] =  preferences.getFloat("gyrobias0");
    calib.gyroBias[1] =  preferences.getFloat("gyrobias1");
    calib.gyroBias[2] =  preferences.getFloat("gyrobias2");

    err = IMU.init(calib, IMU_ADDRESS);
    if(err != 0) {
      Serial.print("Error initializing IMU: ");
      Serial.println(err);
      while (true) {
        ;
      }
    }

    Serial.println("Setting IMU ranges.");
    err = IMU.setGyroRange(512);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
    err = IMU.setAccelRange(4);       //THESE TWO SET THE GYRO RANGE TO ±128 DPS AND THE ACCELEROMETER RANGE TO ±4g

    if (err != 0) {
      Serial.print("Error Setting range: ");
      Serial.println(err);
      while (true) {
        ;
      }
    }
  }

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

  // Serial.print("Accelerometer and Guro range : ");
  // Serial.print(IMU.getAccelRange());
  // Serial.print(", ");
  // Serial.println(IMU.getGyroRange());

  IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);

  double pitch = 0.0;
  double roll = 0.0;
  double yaw = 0.0;

  filter.begin(MAGEWICKFILTER); //0.2f

  roll  = atan2((filter.getQuatW() * filter.getQuatX() + filter.getQuatY() * filter.getQuatZ()), 0.5 - (filter.getQuatX() * filter.getQuatX() + filter.getQuatY() * filter.getQuatY()));
  pitch = asin(2.0 * (filter.getQuatW() * filter.getQuatY() - filter.getQuatX()* filter.getQuatZ()));
  yaw   = atan2((filter.getQuatX() * filter.getQuatY() + filter.getQuatW() * filter.getQuatZ()), 0.5 - (filter.getQuatY() * filter.getQuatY() + filter.getQuatZ() * filter.getQuatZ()));
  roll *= 180.0 / PI;   //X
  pitch *= 180.0 / PI;  //Y
  yaw *= 180.0 / PI;    //Z

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(yaw);

  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();


// //try matrix methods
//   prevTime = millis();
//   startupTime= millis();
//   // Initialize DCM
//   init_DCM();
}

void IMUCalibrate(){

  int err = 0;  
  err = IMU.init(calib, IMU_ADDRESS);
  if(err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  Serial.println("FastIMU Calibrate");
  Serial.println("Keep IMU level.");
  delay(1000);

  IMU.calibrateAccelGyro(&calib);

  preferences.putFloat("accelbias0", calib.accelBias[0]);
  preferences.putFloat("accelbias1", calib.accelBias[1]);
  preferences.putFloat("accelbias2", calib.accelBias[2]);
  preferences.putFloat("gyrobias0", calib.gyroBias[0]);
  preferences.putFloat("gyrobias1", calib.gyroBias[1]);
  preferences.putFloat("gyrobias2", calib.gyroBias[2]);

  err = IMU.init(calib, IMU_ADDRESS);
  if(err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

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

  Serial.println("Setting IMU ranges.");
  err = IMU.setGyroRange(512);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  err = IMU.setAccelRange(4);       //THESE TWO SET THE GYRO RANGE TO ±128 DPS AND THE ACCELEROMETER RANGE TO ±4g

  Serial.println("Calibration done!");
}

void GPSCalcs(){
  static double lastspeed;
  static double alpha1 = 0.05;
  uint32_t time = 0;
  static uint32_t last_time = 0;
  static double lastyawRate = 0;
  static double lastaccelXGPS = 0;

  time = ble_floatValues[TIME];
  //GPS derived speed, course and yaw rate
  if ((time != last_time)  && (time != 0)){
    last_time = time;
    
    speed =  (double)ble_floatValues[SPEEDMS];
    course = (double)ble_floatValues[CRS];
    yawRate = (double)ble_floatValues[YAWRATE];
    accelXGPS = (double)ble_floatValues[ACCELERATION] / 9.81; // acceleration in 'g'

    //acceleration
    accelXGPS = alpha1 * accelXGPS + (1 - alpha1) * lastaccelXGPS; //x
    lastaccelXGPS = accelXGPS;
    //speed
    speed = alpha1 * speed + (1 - alpha1) * lastspeed; //x
    lastspeed = speed;
    //course = alpha1 * course + (1 - alpha1) * lastcourse; //y
    //yaw2Rate
    yawRate = alpha1 * yawRate + (1 - alpha1) * lastyawRate; //y
    lastyawRate = yawRate;

    rot_deg_min = yawRate * 60; //yaw rate in deg/min for instrumentation
    yawRate = yawRate * PI/180; //Yaw rate in radians per sec for calcs
  
    radiusofturn = speed/yawRate;
    centripetalAccel = (speed * speed)/(radiusofturn * 9.81);  // in 'g'
  }
}

void doIMU(){
  double pitch = 0.0;
  double roll = 0.0;
  double yaw = 0.0;
  double magpitch = 0.0;
  double magroll = 0.0;
  double magyaw = 0.0;
  unsigned long imu_currentMillis = millis();

  if (imu_currentMillis - imu_previousMillis >= imu_interval) {
    imu_previousMillis = imu_currentMillis;
    dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    GPSCalcs();  //

    IMU.update();
    IMU.getAccel(&IMUAccel);
    IMU.getGyro(&IMUGyro);

    accX = 2 * IMUAccel.accelX;
    accY = 2 * IMUAccel.accelY;
    accZ = 2 * IMUAccel.accelZ;

        // Apply low-pass filter to accelerometer data
    Accel_Vector[X] = alpha * accX + (1 - alpha) * prevAccel[X]; //x
    Accel_Vector[Y] = alpha * accY + (1 - alpha) * prevAccel[Y]; //y
    Accel_Vector[Z] = alpha * accZ+ (1 - alpha) * prevAccel[Z]; //z
   
    prevAccel[X] = Accel_Vector[X];
    prevAccel[Y] = Accel_Vector[Y];
    prevAccel[Z] = Accel_Vector[Z];

    // Serial.print("Acceleration vectors: ");
    // Serial.print(Accel_Vector[X]);
    // Serial.print(", ");
    // Serial.print(Accel_Vector[Y]);
    // Serial.print(", ");
    // Serial.println(Accel_Vector[Z]);

    // Normalize accelerometer vector
    // float accelMagnitude = sqrt(Accel_Vector[0] * Accel_Vector[0] + Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]);
    // Accel_Vector[X] /= accelMagnitude;
    // Accel_Vector[Y] /= accelMagnitude;
    // Accel_Vector[Z] /= accelMagnitude;

    //replace y acceleration with centripeatal force
    if (speed > LOWERSPEEDLIMIT) {
      Accel_Vector[Y] = Accel_Vector[Y] - centripetalAccel;
      Accel_Vector[X] = Accel_Vector[X] + accelXGPS;
    }

    accXangle = Accel_Vector[X];
    accYangle = Accel_Vector[Y];
    accZangle = Accel_Vector[Z];

        //Apply low-pass filter to gyroscope data
    Gyro_Vector[X] = alpha * (IMUGyro.gyroX) + (1 - alpha) * prevGyro[X]; //x
    Gyro_Vector[Y] = alpha * (IMUGyro.gyroY) + (1 - alpha) * prevGyro[Y]; //y
    Gyro_Vector[Z] = alpha * (IMUGyro.gyroZ) + (1 - alpha) * prevGyro[Z]; //Z

    prevGyro[X] = Gyro_Vector[X];
    prevGyro[Y] = Gyro_Vector[Y];
    prevGyro[Z] = Gyro_Vector[Z];

    gyroXangle = Gyro_Vector[X];
    gyroYangle = Gyro_Vector[Y];
    gyroZangle = Gyro_Vector[Z];

    //gpointer
    zacceleration = Accel_Vector[Z];
    
    //raw roll and pitch
    roll  = atan2(Accel_Vector[1], Accel_Vector[2]) * 180/PI;  // Roll (atan2(Y, Z))
    pitch = atan2(-Accel_Vector[0], sqrt(Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2])) * 180/PI;  // Pitch (atan2(-X, sqrt(Y² + Z²)))
    yaw = 0;
    if (speed > LOWERSPEEDLIMIT)  yaw = course; // from GPS
    
    //Madgewick
    if (aiDisplaysource == 4){
      filter.updateIMU(Gyro_Vector[X], Gyro_Vector[Y], Gyro_Vector[Z], Accel_Vector[X], Accel_Vector[Y], Accel_Vector[Z]);
      magroll  = atan2((filter.getQuatW() * filter.getQuatX() + filter.getQuatY() * filter.getQuatZ()), 0.5 - (filter.getQuatX() * filter.getQuatX() + filter.getQuatY() * filter.getQuatY()));
      magpitch = asin(2.0 * (filter.getQuatW() * filter.getQuatY() - filter.getQuatX()* filter.getQuatZ()));
      magyaw   = atan2((filter.getQuatX() * filter.getQuatY() + filter.getQuatW() * filter.getQuatZ()), 0.5 - (filter.getQuatY() * filter.getQuatY() + filter.getQuatZ() * filter.getQuatZ()));
      
      magAngleX = magroll * 180.0 / PI;
      magAngleY = magpitch * 180.0 / PI;
      magAngleZ = magyaw * 180.0 / PI;

      // GPS yaw correction
      if (speed > LOWERSPEEDLIMIT) {
          // // Apply GPS yaw correction if speed is above threshold
          // double yawError = course - (magyaw * 180.0 / PI);
          // // Ensure yaw stays within -180 to 180 degrees
          // if (yawError > 180) yawError -= 360;
          // if (yawError < -180) yawError += 360;

          // // Blend the Madgwick yaw with the GPS yaw, gradually correcting it
          // course += alpha1 * yawError;  // Adjust the factor (0.05) as necessary
          // magAngleZ = course;
      }
      pitchdifference = abs(pitch-magAngleY) + abs(roll - magAngleX);
    }
    //********************

    //Complimentary Filter
    if (aiDisplaysource == 2){
        compAngleX = compfiltervariable * (compAngleX + Gyro_Vector[X] * dt) + (1 - compfiltervariable) * roll; // Calculate the angle using a Complimentary filter
        compAngleY = compfiltervariable * (compAngleY + Gyro_Vector[Y] * dt) + (1 - compfiltervariable) * pitch;
        compAngleZ = compfiltervariable * (compAngleZ + Gyro_Vector[Z] * dt) + (1 - compfiltervariable) * course;
        // ptch difference display
        pitchdifference = abs(pitch-compAngleY) + abs(roll - compAngleX);
    }

    //********************

    //kalman filter
    if (aiDisplaysource == 3){
      kalAngleX = kalmanX.getAngle(roll, Gyro_Vector[X], dt); // Calculate the angle using a Kalman filter
      kalAngleY = kalmanY.getAngle(pitch, Gyro_Vector[Y], dt); // Pitch , roll and yaw are from accelerometer
      kalAngleZ = kalmanZ.getAngle(course, Gyro_Vector[Z], dt);   // Z is from gps

      kgyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
      kgyroYangle += kalmanY.getRate() * dt;
      kgyroZangle += kalmanZ.getRate() * dt;

      pitchdifference = abs(pitch-kgyroYangle) + abs(roll - kgyroXangle);
    }

    switch (aiDisplaysource){
      case 1:
        imupitch = gyroYangle;
        imuroll = gyroXangle;
        imuyaw = gyroZangle;
        break;
      case 2:
        imupitch = compAngleY;
        imuroll = compAngleX;
        imuyaw = compAngleZ;
        break;
      case 3:
        imupitch = kalAngleY;
        imuroll = kalAngleX;
        imuyaw = kalAngleZ;
        break;
      case 4:
        imupitch = magAngleY;
        imuroll = magAngleX;
        imuyaw = magAngleZ;
        break;
    }
  }
  // Other non-blocking tasks can go here
}

//#endif

// //******************************************************************
// //DCM Matrix Code

// // Initialize DCM matrix to identity
// void init_DCM() {
//   DCM_Matrix[0][0] = 1.0; DCM_Matrix[0][1] = 0.0; DCM_Matrix[0][2] = 0.0;
//   DCM_Matrix[1][0] = 0.0; DCM_Matrix[1][1] = 1.0; DCM_Matrix[1][2] = 0.0;
//   DCM_Matrix[2][0] = 0.0; DCM_Matrix[2][1] = 0.0; DCM_Matrix[2][2] = 1.0;
// }

// void doIMU1(){ 

//   unsigned long imu_currentMillis = millis();
//   if (imu_currentMillis - imu_previousMillis >= imu_interval) {

//     IMU.update();
//     IMU.getAccel(&IMUAccel);
//     IMU.getGyro(&IMUGyro);

//     // Serial.print("Raw Acceleration vectors: ");
//     // Serial.print(IMUAccel.accelX);
//     // Serial.print(", ");
//     // Serial.print(IMUAccel.accelY);
//     // Serial.print(", ");
//     // Serial.println(IMUAccel.accelZ);

//     imu_previousMillis = imu_currentMillis;
//     unsigned long currTime = micros();
//     dt = (currTime - prevTime) / 1000000.0;  // Time step in seconds
//     prevTime = currTime;
    
//     // Apply low-pass filter to accelerometer data
//     Accel_Vector[0] = alpha * IMUAccel.accelX + (1 - alpha) * prevAccel[0];
//     Accel_Vector[1] = alpha * IMUAccel.accelY + (1 - alpha) * prevAccel[1];
//     Accel_Vector[2] = alpha * IMUAccel.accelZ + (1 - alpha) * prevAccel[2];
   
//     prevAccel[0] = Accel_Vector[0];
//     prevAccel[1] = Accel_Vector[1];
//     prevAccel[2] = Accel_Vector[2];

//     zacceleration = Accel_Vector[2];

//     // Normalize accelerometer vector
//     float accelMagnitude = sqrt(Accel_Vector[0] * Accel_Vector[0] + Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]);
//     Accel_Vector[0] /= accelMagnitude;
//     Accel_Vector[1] /= accelMagnitude;
//     Accel_Vector[2] /= accelMagnitude;

//     Serial.print("LP Acceleration vectors: ");
//     Serial.print(Accel_Vector[0]);
//     Serial.print(", ");
//     Serial.print(Accel_Vector[1]);
//     Serial.print(", ");
//     Serial.println(Accel_Vector[2]);

//     gyroX = IMUGyro.gyroX;
//     gyroY = IMUGyro.gyroY;
//     gyroZ = IMUGyro.gyroZ;

//     //Apply low-pass filter to gyroscope data
//     Gyro_Vector[0] = alpha * (IMUGyro.gyroX) + (1 - alpha) * prevGyro[0];
//     Gyro_Vector[1] = alpha * (IMUGyro.gyroY) + (1 - alpha) * prevGyro[1];
//     Gyro_Vector[2] = alpha * (IMUGyro.gyroZ) + (1 - alpha) * prevGyro[2];

//     prevGyro[0] = Gyro_Vector[0];
//     prevGyro[1] = Gyro_Vector[1];
//     prevGyro[2] = Gyro_Vector[2];

//     Serial.print("LP Gyro vectors: ");
//     Serial.print(Gyro_Vector[0]);
//     Serial.print(", ");
//     Serial.print(Gyro_Vector[1]);
//     Serial.print(", ");
//     Serial.println(Gyro_Vector[2]);

//     // Update DCM matrix with gyroscope integration
//     updateDCMMatrix();
    
//     if (millis() - startupTime > startupDelay) {
//       // Apply drift correction after initial delay
//       driftCorrection();
//     }
//     //printMatrix(DCM_Matrix, 3, 3);

//     double roll = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);  // Roll from DCM (X-axis)
//     double pitch = -asin(DCM_Matrix[2][0]);  // Pitch from DCM (Y-axis)

//     // Convert to degrees
//     roll = roll * (180.0 / PI);
//     pitch = pitch * (180.0 / PI);
//     //yaw = yaw * (180.0 / PI);  // Yaw will drift over time without correction

//     //Print the filtered angles
//     Serial.print("Roll: ");
//     Serial.print(roll);
//     Serial.print(", Pitch: ");
//     Serial.println(pitch);
//     // Serial.print(", Yaw (gyroscope only): ");
//     // Serial.println(yaw);

//     imupitch = pitch;
//     imuroll = roll;
//     //imuyaw = yaw;
//   }
// }

// void updateDCMMatrix() {
//     float Omega_x = Gyro_Vector[0] * dt * DEG_TO_RAD;  // Roll rate * delta time
//     float Omega_y = Gyro_Vector[1] * dt* DEG_TO_RAD;  // Pitch rate * delta time
//     float Omega_z = Gyro_Vector[2] * dt* DEG_TO_RAD;  // Yaw rate * delta time

//     // Create an update matrix (using angular velocities for a small rotation)
//     Update_Matrix[0][0] = 0.0;
//     Update_Matrix[0][1] = -Omega_z;
//     Update_Matrix[0][2] = Omega_y;

//     Update_Matrix[1][0] = Omega_z;
//     Update_Matrix[1][1] = 0.0;
//     Update_Matrix[1][2] = -Omega_x;

//     Update_Matrix[2][0] = -Omega_y;
//     Update_Matrix[2][1] = Omega_x;
//     Update_Matrix[2][2] = 0.0;

//     // Multiply DCM by the update matrix
//     matrixMultiply(DCM_Matrix, Update_Matrix, Temporary_Matrix);

//     // Apply a scaled update (don't just add the Temporary_Matrix to DCM directly)
//     for (int i = 0; i < 3; i++) {
//         for (int j = 0; j < 3; j++) {
//             DCM_Matrix[i][j] += Temporary_Matrix[i][j];  // Add small rotations to DCM
//         }
//     }

//     // Normalize the DCM matrix after the update to maintain orthogonality
//     normalizeDCM();
// }

// void driftCorrection() {
//     // Calculate roll and pitch from the accelerometer (acceleration matrix)
//     double accel_roll = atan2(Accel_Vector[1], Accel_Vector[2]);  // Roll (atan2(Y, Z))
//     double accel_pitch = atan2(-Accel_Vector[0], sqrt(Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]));  // Pitch (atan2(-X, sqrt(Y² + Z²)))

//     Serial.print("accel_roll: ");
//     Serial.println(accel_roll * (180.0 / PI), 6);  // Print in degrees
//     Serial.print("accel_pitch: ");
//     Serial.println(accel_pitch * (180.0 / PI), 6);  // Print in degrees

//     // Calculate roll and pitch from the DCM matrix
//     double dcm_roll = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);  // Roll from DCM (X-axis)
//     double dcm_pitch = -asin(DCM_Matrix[2][0]);  // Pitch from DCM (Y-axis)

//     Serial.print("dcm_roll: ");
//     Serial.println(dcm_roll * (180.0 / PI), 6);  // Print in degrees
//     Serial.print("dcm_pitch: ");
//     Serial.println(dcm_pitch * (180.0 / PI), 6);  // Print in degrees

//     // Calculate error between accelerometer and DCM for roll and pitch (errors are already in radians)
//     double roll_error = -(dcm_roll - accel_roll);
//     double pitch_error = -(dcm_pitch - accel_pitch);

//     // Print out roll and pitch errors for debugging (in degrees for readability)
//     Serial.print("Roll Error: ");
//     Serial.println(roll_error * (180.0 / PI), 6);  // Print in degrees
//     Serial.print("Pitch Error: ");
//     Serial.println(pitch_error * (180.0 / PI), 6);  // Print in degrees

//     roll_error = roll_error * 0.05;
//     pitch_error = pitch_error * 0.05;

// //     // Create a small rotation matrix for roll correction (rotation around X-axis)
//     double roll_correction_matrix[3][3] = {
//         {1.0,            0.0,               0.0},
//         {0.0, cos(roll_error), -sin(roll_error)},
//         {0.0, sin(roll_error),  cos(roll_error)}
//     };

//     // Create a small rotation matrix for pitch correction (rotation around Y-axis)
//     double pitch_correction_matrix[3][3] = {
//         {cos(pitch_error),  0.0, sin(pitch_error)},
//         {0.0,              1.0, 0.0},
//         {-sin(pitch_error), 0.0, cos(pitch_error)}
//     };

//     // Temporary matrix to hold the result of the first multiplication
//     double temp_matrix[3][3];

//     // Multiply the DCM by the roll correction matrix
//     matrixMultiply(DCM_Matrix, roll_correction_matrix, temp_matrix);

//     // Multiply the result by the pitch correction matrix
//     matrixMultiply(temp_matrix, pitch_correction_matrix, DCM_Matrix);

//     // Normalize the DCM matrix to maintain orthogonality
//     normalizeDCM();
//  }

// void matrixMultiply(double a[3][3], double b[3][3], double result[3][3]) {
//   for (int i = 0; i < 3; i++) {
//     for (int j = 0; j < 3; j++) {
//       result[i][j] = 0;
//       for (int k = 0; k < 3; k++) {
//         result[i][j] += a[i][k] * b[k][j];
//       }
//     }
//   }
// }

// void normalizeDCM() {
//     // Calculate the error between the first two rows
//     double error = -0.5 * (DCM_Matrix[0][0] * DCM_Matrix[1][0] +
//                           DCM_Matrix[0][1] * DCM_Matrix[1][1] +
//                           DCM_Matrix[0][2] * DCM_Matrix[1][2]);

//     // Correct the first two rows to maintain orthogonality
//     for (int i = 0; i < 3; i++) {
//         DCM_Matrix[0][i] += error * DCM_Matrix[1][i];
//         DCM_Matrix[1][i] += error * DCM_Matrix[0][i];
//     }

//     // Normalize the first row
//     double row1_mag = 0.5 * (3 - (DCM_Matrix[0][0] * DCM_Matrix[0][0] +
//                                  DCM_Matrix[0][1] * DCM_Matrix[0][1] +
//                                  DCM_Matrix[0][2] * DCM_Matrix[0][2]));
//     for (int i = 0; i < 3; i++) {
//         DCM_Matrix[0][i] *= row1_mag;
//     }

//     // Normalize the second row
//     double row2_mag = 0.5 * (3 - (DCM_Matrix[1][0] * DCM_Matrix[1][0] +
//                                  DCM_Matrix[1][1] * DCM_Matrix[1][1] +
//                                  DCM_Matrix[1][2] * DCM_Matrix[1][2]));
//     for (int i = 0; i < 3; i++) {
//         DCM_Matrix[1][i] *= row2_mag;
//     }

//     // Recalculate the third row as the cross-product of the first two rows
//     DCM_Matrix[2][0] = DCM_Matrix[0][1] * DCM_Matrix[1][2] - DCM_Matrix[0][2] * DCM_Matrix[1][1];
//     DCM_Matrix[2][1] = DCM_Matrix[0][2] * DCM_Matrix[1][0] - DCM_Matrix[0][0] * DCM_Matrix[1][2];
//     DCM_Matrix[2][2] = DCM_Matrix[0][0] * DCM_Matrix[1][1] - DCM_Matrix[0][1] * DCM_Matrix[1][0];
// }


// void printMatrix(double matrix[][3], int nRows, int nCols) {
//     Serial.println("Matrix Contents:");
    
//     // Loop through each row and column of the NxN matrix
//     for (int i = 0; i < nRows; i++) {
//         for (int j = 0; j < nCols; j++) {
//             Serial.print(matrix[i][j], 6);  // Print each element with 6 decimal precision
//             Serial.print("\t");  // Tab space between elements
//         }
//         Serial.println();  // Newline after each row
//     }
// }


