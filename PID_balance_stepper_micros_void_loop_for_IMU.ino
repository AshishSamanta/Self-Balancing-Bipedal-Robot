#include "stepper_driver_MKS_TMC2160-OC.h"
#include "servo_ik.h"
#include "imu_sensor_ISM330DHCX.h"
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

double imu_freq = 0;
// Timing diagnostics (in microseconds)
volatile unsigned long loop_dt = 0;
volatile unsigned long servo_dt = 0;
volatile unsigned long imu_dt   = 0;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 10; // ms
double body_height_command = 330;

struct STATE {
  double x_dot;
  double theta;
  double theta_dot;
  double alpha;
  double alpha_dot;
};
STATE state;

// --- PID gains ---
float Kp_theta = 0.5;
float Kd_theta = 0.0000;
float Ki_theta = 0.00000;
float theta_integral = 0.0;
double theta_filtered = 0.0;

float Kp_vel = 0.0;
float Kd_vel = 0.0000;
float Ki_vel = 0.000000;
double vel_integral = 0.0;
double vel_prev_error = 0.0;

double freq = 1000;

// ----- FreeRTOS queue for IMU samples -----
static QueueHandle_t imuQueue = NULL; // holds the latest IMUData (queue length 1)

// shared variables
float roll_cmd_global = 0;
float body_height_global = 330;
portMUX_TYPE servoMux = portMUX_INITIALIZER_UNLOCKED;

// ================= SERVO TASK =================
void ServoTask(void *pvParameters) {
  unsigned long prev = micros();
  while (true) {
    unsigned long now = micros();
    servo_dt = now - prev;
    prev = now;

    float roll, height;
    portENTER_CRITICAL(&servoMux);
    roll = roll_cmd_global;
    height = body_height_global;
    portEXIT_CRITICAL(&servoMux);

    ServoIK(roll, height);
    vTaskDelay(10 / portTICK_PERIOD_MS); // ~100 Hz
  }
}

// ================= CONTROL TASK =================
void ControlTask(void *pvParameters) {
  unsigned long prev = micros();
  while (true) {
    unsigned long now = micros();
    loop_dt = now - prev;
    prev = now;

    double loop_start_time = millis();

    portENTER_CRITICAL(&servoMux);
    roll_cmd_global = 0;
    body_height_global = 330;
    portEXIT_CRITICAL(&servoMux);

    // Try to read latest IMU sample (non-blocking)
    IMUData imuData;
    bool gotIMU = false;
    if (imuQueue && xQueueReceive(imuQueue, &imuData, 0) == pdPASS) {
      gotIMU = true;
    }

    if (gotIMU) {
      double accel = imuData.linearAccX;
      double omega_gyro = -imuData.angularVelZ;
      double lin_vel_cmd = 0;
      double ang_vel_cmd = 0;
      if (abs(lin_vel_cmd) <= 1) lin_vel_cmd = 0;
      if (abs(ang_vel_cmd) <= 1) ang_vel_cmd = 0;

      // Velocity PID
      double vel_error = lin_vel_cmd - drive_vel.lin_vel;
      vel_integral += vel_error * (1.0 / freq);
      vel_integral = constrain(vel_integral, -1, 1);
      double vel_derivative = (vel_error - vel_prev_error) / (1.0 / freq);
      vel_prev_error = vel_error;
      double vel_pid_output = Kp_vel * vel_error
                            + Ki_vel * vel_integral
                            + Kd_vel * vel_derivative;

      // Target Pitch
      double target_pitch = constrain(vel_pid_output, -3.0, 3.0);

      // Angle PID
      state.theta = imuData.pitch;
      state.theta_dot = imuData.angularVelY;

      unsigned long current_print_time = millis();
      if((current_print_time - lastPrintTime) > printInterval){
        // Serial.print("imu_dt: ");
        Serial.print(imu_freq);Serial.print("\t");
        // Serial.print(" ");
        Serial.print(10); Serial.print("\t");
        Serial.print(-10); Serial.print("\t");
        // Serial.print(" loop_dt: ");
        // Serial.print(loop_dt); Serial.print("\t");
        // Serial.print(" servo_dt: ");
        // Serial.print(servo_dt); Serial.print("\t");
        // Serial.print(" ");
        Serial.print(state.theta,4); Serial.print("\t");
        Serial.println(state.theta_dot,4);
        lastPrintTime = current_print_time;
      }
      

      double error = target_pitch - state.theta;
      theta_integral += error * (1.0 / freq);
      theta_integral = constrain(theta_integral, -0.5, 0.5);
      double derivative = -state.theta_dot;
      double theta_output = Kp_theta * error
                          + Kd_theta * derivative
                          + Ki_theta * theta_integral;

      // Motor Commands
      double steering_output = ang_vel_cmd;
      double left_omega = constrain(theta_output + steering_output, -5.56, 5.56);
      double right_omega = constrain(theta_output - steering_output, -5.56, 5.56);
      motor_driver_set_omega(left_omega, right_omega);
      // Serial.print(left_omega);
      // Serial.println(right_omega);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ================= SETUP =================
void setup() {
  Wire.begin(21, 22, 400000);
  Serial.begin(921600);

  setupServoIK();
  setupIMU();
  delay(1000);
  motor_driver_init();

  // create IMU queue (latest sample only)
  imuQueue = xQueueCreate(1, sizeof(IMUData));
  if (!imuQueue) {
    Serial.println("Failed to create IMU queue!");
  }

  // Servo Task on Core 1
  xTaskCreatePinnedToCore(
    ServoTask, "ServoTask", 4096,
    NULL, 1, NULL, 1
  );

  // Control Task on Core 1
  xTaskCreatePinnedToCore(
    ControlTask, "ControlTask", 8192,
    NULL, 2, NULL, 1
  );
}

// ================= HIGH-FREQ IMU LOOP =================
void loop() {
  static unsigned long prevMicros = 0;
  const unsigned long imuInterval = 333; // 3 kHz (333 µs)
  static IMUData imuSample;

  unsigned long now = micros();
  if (now - prevMicros >= imuInterval) {
    imu_dt = now - prevMicros;
    prevMicros = now;

    bool valid = false;
    IMUData imuSampleTemp = updateIMU(valid);

    if (valid) {
        imuSample = imuSampleTemp;
        if (imuQueue) {
            xQueueOverwrite(imuQueue, &imuSample);
        }
      imu_freq = 1000000/(imu_dt);
    }
  }
}
