#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa imu;

float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

float gx, gy, gz;
float ax, ay, az;

struct IMUData {
    float linearAccX;
    float roll;
    float pitch;
    float yaw;
    float angularVelX;
    float angularVelY;
    float angularVelZ;  // in rad/s
  };
  
void calibrateGyro(int numSamples = 500) {
  Serial.println("Calibrating gyroscope... Keep the sensor still.");

  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < numSamples; i++) {
    imu.gyroUpdate();
    sumX += imu.gyroX();
    sumY += imu.gyroY();
    sumZ += imu.gyroZ();
    delay(5);  // wait for next reading
  }

  gyroBiasX = sumX / numSamples;
  gyroBiasY = sumY / numSamples;
  gyroBiasZ = sumZ / numSamples;

  Serial.println("Calibration complete.");
  Serial.printf("Gyro Bias: X=%.2f, Y=%.2f, Z=%.2f\n\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

void setupIMU() {
    delay(100);
    Wire.begin(); // SDA = GPIO21, SCL = GPIO22 on ESP32
    imu.setWire(&Wire);
    imu.beginAccel();
    imu.beginGyro();
    imu.beginMag(); // even if not working, harmless
  
    delay(100);
  
    calibrateGyro();
}

IMUData updateIMU() {
  float gx = 0, gy = 0, gz = 0;
  float ax = 0, ay = 0, az = 0;

  for (int i = 0; i < 10; i++) {
    imu.gyroUpdate();
    gx += imu.gyroX() - gyroBiasX;
    gy += imu.gyroY() - gyroBiasY;
    gz += imu.gyroZ() - gyroBiasZ;
  }
  gx *= 3.1415 / (10 * 180);
  gy *= 3.1415 / (10 * 180);
  gz *= 3.1415 / (10 * 180);

  for (int i = 0; i < 10; i++) {
    imu.accelUpdate();
    ax += imu.accelX();
    ay += imu.accelY();
    az += imu.accelZ();
  }
  ax *= 9.81 / 10;
  ay *= 9.81 / 10;
  az *= 9.81 / 10;

  float roll  = atan2(ay, -az) * 180.0 / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float yaw = 0;
  IMUData result = {ax, roll, pitch, yaw, gx, gy, gz};
  return result;
}

#endif
