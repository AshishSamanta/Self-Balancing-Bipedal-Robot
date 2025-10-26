#ifndef IMU_SENSOR_ISM330DHCX_H
#define IMU_SENSOR_ISM330DHCX_H

#include <SPI.h>
#include "SparkFun_ISM330DHCX.h"

// ---------- STRUCT DEFINITION ----------
struct IMUData {
  float linearAccX;
  float roll;
  float pitch;
  float yaw;
  float angularVelX;
  float angularVelY;
  float angularVelZ;  // in rad/s
};

// ---------- INTERNAL GLOBALS ----------
static SparkFun_ISM330DHCX_SPI myISM;
static sfe_ism_data_t accelData;
static sfe_ism_data_t gyroData;

static byte chipSelect = 26;

static float filtAccelX = 0, filtAccelY = 0, filtAccelZ = 0;
static float filtGyroX = 0, filtGyroY = 0, filtGyroZ = 0;

static float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

static unsigned long prevMicros = 0;

static const float alphaAcc = 0.02;   // accel anti smoothing
static const float alphaGyro = 0.02; // gyro anti smoothing

static float roll_rad, pitch_rad;
static float loopFreq = 0;

// ---------- FUNCTION DEFINITIONS ----------

void setupIMU() {
  SPI.begin(18, 19, 23); // SCK, MISO, MOSI
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);

  if (!myISM.begin(chipSelect)) {
    Serial.println("ISM330DHCX not detected!");
    while (1);
  }

  myISM.deviceReset();
  while (!myISM.getDeviceReset()) delay(1);

  myISM.setDeviceConfig();
  myISM.setBlockDataUpdate();

  // Accelerometer
  myISM.setAccelDataRate(ISM_XL_ODR_3332Hz);
  myISM.setAccelFullScale(ISM_8g);

  // Gyroscope
  myISM.setGyroDataRate(ISM_GY_ODR_3332Hz);
  myISM.setGyroFullScale(ISM_2000dps);

  prevMicros = micros();
  Serial.println("IMU setup complete.");
}

void calibrateGyro(int numSamples = 500) {
  long sumX = 0, sumY = 0, sumZ = 0;
  int count = 0;

  Serial.println("Calibrating gyroscope... Keep IMU stationary.");

  while (count < numSamples) {
    if (myISM.checkStatus()) {
      myISM.getGyro(&gyroData);
      sumX += gyroData.xData;
      sumY += gyroData.yData;
      sumZ += gyroData.zData;
      count++;
    }
  }

  gyroOffsetX = sumX / (float)numSamples;
  gyroOffsetY = sumY / (float)numSamples;
  gyroOffsetZ = sumZ / (float)numSamples;

  Serial.print("Gyro calibration done. Offsets: ");
  Serial.print("X="); Serial.print(gyroOffsetX);
  Serial.print(" Y="); Serial.print(gyroOffsetY);
  Serial.print(" Z="); Serial.println(gyroOffsetZ);
}

IMUData updateIMU(bool &valid) {
  IMUData imuOut = {};
  valid = false;

  if (!myISM.checkStatus()) {
    return imuOut;
  }

  myISM.getAccel(&accelData);
  myISM.getGyro(&gyroData);

  // --- Gyroscope (mdps → rad/s) ---
  float gX = (gyroData.xData - gyroOffsetX) * 0.001f;
  float gY = (gyroData.yData - gyroOffsetY) * 0.001f;
  float gZ = (gyroData.zData - gyroOffsetZ) * 0.001f;

  // --- Accelerometer (mg → m/s²) ---
  float aX = accelData.xData * 0.001f * 9.80665f;
  float aY = accelData.yData * 0.001f * 9.80665f;
  float aZ = accelData.zData * 0.001f * 9.80665f;

  // --- Exponential filtering ---
  filtAccelX = alphaAcc * aX + (1 - alphaAcc) * filtAccelX;
  filtAccelY = alphaAcc * aY + (1 - alphaAcc) * filtAccelY;
  filtAccelZ = alphaAcc * aZ + (1 - alphaAcc) * filtAccelZ;

  filtGyroX = alphaGyro * gX + (1 - alphaGyro) * filtGyroX;
  filtGyroY = alphaGyro * gY + (1 - alphaGyro) * filtGyroY;
  filtGyroZ = alphaGyro * gZ + (1 - alphaGyro) * filtGyroZ;

  // --- Roll and Pitch ---
  roll_rad  = -atan2f(-filtAccelX, sqrtf(filtAccelY*filtAccelY + filtAccelZ*filtAccelZ)); //interchanged with pitch with -ve sign
  pitch_rad = atan2f(filtAccelY, filtAccelZ); //interchanged with roll 

  // --- Timing ---
  unsigned long currentMicros = micros();
  float dt = (currentMicros - prevMicros) / 1e6f;
  prevMicros = currentMicros;
  loopFreq = 1.0f / dt;

  // --- Fill output struct ---
  imuOut.linearAccX = -filtAccelY;
  imuOut.roll  = roll_rad * RAD_TO_DEG;
  imuOut.pitch = pitch_rad * RAD_TO_DEG;
  imuOut.yaw   = 0.0f; // optional yaw integration later
  imuOut.angularVelX = -filtGyroY;
  imuOut.angularVelY = filtGyroX;
  imuOut.angularVelZ = filtGyroZ;
  // Serial.print(imuOut.angularVelX); Serial.print("\t");
  valid = true;
  return imuOut;
}

#endif