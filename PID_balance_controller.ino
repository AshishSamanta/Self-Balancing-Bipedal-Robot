#include "stepper_driver_MKS_TMC2160-OC_acceleration.h"
#include "servo_ik.h"
#include "imu_sensor_ISM330DHCX.h"
#include <Wire.h> //i2c protocols header file
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "HC-12.h"

double imu_freq = 0;
double freq = 1000; // approximate measured frq is 1000hz

float Acceleration_limits = 50;// rad/sec^2

volatile unsigned long loop_dt = 0;
volatile unsigned long servo_dt = 0;
volatile unsigned long imu_dt   = 0;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 10; // ms

// current_command
struct Command {
  double lin_vel_cmd;
  double ang_vel_cmd;
  double hull_roll_cmd;
  double hull_pitch_cmd;
  double body_height_cmd;  
  double x_pos_cmd;      
};
Command cmd = {0, 0, 0, 0, 330, 30};

struct STATE {
  double x_dot;
  double theta;
  double theta_dot;
  double alpha;
  double alpha_dot;
};
STATE state;

// --- PID gains ---
float Kp_theta = 10;
float Kd_theta = 1.25;
float Ki_theta = 0.0005;
float theta_integral = 0.0;
double theta_filtered = 0.0;

double kp_theta_height_factor = 0.045;
double Kd_theta_height_factor = 0.00608;
double ki_theta_height_factor = 0.0001;

float Kp_vel = -2.0;
float Kd_vel = 0.0;
float Ki_vel = -0.5;
double vel_integral = 0.0;
double vel_prev_error = 0.0;


// ----- FreeRTOS queue for IMU samples -----
static QueueHandle_t imuQueue = NULL; // holds the latest IMUData (queue length 1)

// commands 

//================ disable/enable movement =================
// --- Movement disable logic ---
bool movement_disabled = true;
bool button_was_pressed = false;
bool toggled_this_press = false;
bool state_at_press = false;
unsigned long press_start_time = 0;

// --- Reset roll/pitch smoothing logic ---
bool reset_roll_pitch_active = false;
unsigned long reset_roll_pitch_start = 0;
const unsigned long RESET_ROLL_PITCH_DURATION = 3000;  // ms

// --- Timing guard for reset suppression ---
unsigned long movement_last_disabled_time = 0;   // timestamp when disable triggered
const unsigned long RESET_BLOCK_DURATION = 2000; // block reset for 2s after disable

void handleDisableButton() {
  bool pressed = ctrlData.disable_movement;

  static bool button_was_pressed = false;
  static unsigned long press_start_time = 0;
  static bool toggled_during_press = false;

  const unsigned long HOLD_DURATION = 1000;  // 1 s hold to toggle

  if (pressed && !button_was_pressed) {
    // Button just pressed
    press_start_time = millis();
    toggled_during_press = false;
    button_was_pressed = true;
  }

  // Only toggle once per hold
  if (pressed && !toggled_during_press && (millis() - press_start_time >= HOLD_DURATION)) {
    movement_disabled = !movement_disabled;
    toggled_during_press = true;  // prevent repeat toggle until released

    Serial.print("[HC12] Movement mode toggled -> ");
    Serial.println(movement_disabled ? "DISABLED" : "ENABLED");
  }

  // Reset only after button is released
  if (!pressed && button_was_pressed) {
    button_was_pressed = false;
    toggled_during_press = false;
  }
}

void handleResetRollPitch() {
  unsigned long now = millis();

  // Block reset if movement was recently disabled
  if (movement_disabled || (now - movement_last_disabled_time < RESET_BLOCK_DURATION)) {
    reset_roll_pitch_active = false;
    return;
  }

  // Trigger reset when signal appears
  if (ctrlData.reset_roll_pitch) {
    reset_roll_pitch_active = true;
    reset_roll_pitch_start = now;
  }

  // Auto-expire after 3 seconds
  if (reset_roll_pitch_active && (now - reset_roll_pitch_start >= RESET_ROLL_PITCH_DURATION)) {
    reset_roll_pitch_active = false;
  }
}


// ================= SERVO TASK =================
void ServoTask(void *pvParameters) {
  unsigned long prev = micros();
  while (true) {
    unsigned long now = micros();
    servo_dt = now - prev;
    prev = now;
    Serial.printf("Before IK: roll=%.3f  pitch=%.3f  lin_vel=%.3f  ang_vel=%.3f     ",
               cmd.hull_roll_cmd, cmd.hull_pitch_cmd,
               cmd.lin_vel_cmd, cmd.ang_vel_cmd);

    ServoIK(cmd.hull_roll_cmd, cmd.hull_pitch_cmd, cmd.body_height_cmd, cmd.x_pos_cmd);

    Serial.printf("After IK: roll=%.3f  pitch=%.3f\n", cmd.hull_roll_cmd, cmd.hull_pitch_cmd);

    vTaskDelay(10 / portTICK_PERIOD_MS); // ~100 Hz
  }
}

// ================= CONTROL TASK =================
void ControlTask(void *pvParameters) {
  unsigned long prev = micros();
  int printCounter = 0;

  while (true) {
    unsigned long now = micros();
    loop_dt = now - prev;
    prev = now;

    double loop_start_time = millis();

    updateHC12();
    handleDisableButton();
    handleResetRollPitch();

    if (movement_disabled) {
      // Gradual decay while disabled
      cmd.lin_vel_cmd *= 0.99;
      cmd.ang_vel_cmd *= 0.99;
      cmd.hull_roll_cmd    *= 0.99;
      cmd.hull_pitch_cmd   *= 0.99;
    } 
    else {
      if (reset_roll_pitch_active) {
        // 3-second smoothing only when allowed
        cmd.hull_roll_cmd  *= 0.99;
        cmd.hull_pitch_cmd *= 0.99;
      }

      cmd.lin_vel_cmd = ctrlData.lin_vel;
      cmd.ang_vel_cmd = ctrlData.ang_vel;

      if (ctrlData.increase_height) cmd.body_height_cmd += 0.1;
      if (ctrlData.decrease_height) cmd.body_height_cmd -= 0.1;
      if (ctrlData.tune_level_up)   cmd.x_pos_cmd += 0.002;
      if (ctrlData.tune_level_down) cmd.x_pos_cmd -= 0.002;

      cmd.body_height_cmd = constrain(cmd.body_height_cmd, 200.0, 330.0);
      cmd.x_pos_cmd       = constrain(cmd.x_pos_cmd, -50.0, 50.0);

      cmd.hull_roll_cmd += ctrlData.roll * 0.002;
      cmd.hull_pitch_cmd += ctrlData.pitch * 0.002;
    }
    

    // Try to read latest IMU sample (non-blocking)
    IMUData imuData;
    bool gotIMU = false;
    if (imuQueue && xQueueReceive(imuQueue, &imuData, 0) == pdPASS) {
      gotIMU = true;
    }

    if (gotIMU) {
      double accel = imuData.linearAccX;
      double omega_gyro = -imuData.angularVelZ;
      // if (abs(cmd.lin_vel_cmd) <= 1) cmd.lin_vel_cmd = 0;
      // if (abs(cmd.ang_vel_cmd) <= 1) cmd.ang_vel_cmd = 0;

      // Velocity PID
      double vel_error = cmd.lin_vel_cmd - drive_vel.lin_vel;
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

      double error = target_pitch - state.theta;
      theta_integral += error * (1.0 / freq);
      theta_integral = constrain(theta_integral, -0.5, 0.5);
      double derivative = -state.theta_dot;
      double theta_output = Kp_theta * error
                          + Kd_theta * derivative;
                          + Ki_theta * theta_integral;

      // Motor Commands
      double steering_output = cmd.ang_vel_cmd;
      double left_accel = constrain(theta_output + steering_output, -Acceleration_limits, Acceleration_limits);
      double right_accel = constrain(theta_output - steering_output, -Acceleration_limits, Acceleration_limits);
      motor_driver_set_accel(left_accel, right_accel);
      motor_driver_update_from_accel();

      // --- Logging ---
      unsigned long current_print_time = millis();
      if((current_print_time - lastPrintTime) > printInterval){

        
        hc12PeriodicSend((double)movement_disabled, (double)reset_roll_pitch_active, (double)cmd.hull_roll_cmd, (double)error);
        lastPrintTime = current_print_time;
      }
      // Serial.println(right_accel);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ================= SETUP =================
void setup() {
  Wire.begin(21, 22, 400000);
  Serial.begin(921600);
  initHC12();

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
