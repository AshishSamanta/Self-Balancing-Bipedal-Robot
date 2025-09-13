#include "motor_driver_cytron.h"
#include "dualshock.h"
#include "servo_ik.h"
#include "imu_sensor.h"

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // in milliseconds
double body_height_command=330;

JOY_CMD joy_cmd = {-1, body_height_cmd, 0.0, 0.0};

////////////////////KALMAN FILTER
double x_est = 0.0;      // Estimated velocity
double P = 1.0;
double Q = 0.05;
double R = 0.05;

double omega_est = 0.0;
double P_omega = 1.0;
double Q_omega = 0.05;
double R_omega = 0.05;
/////////////////////////////////
struct STATE{
  double x_dot;
  double theta;
  double theta_dot;
  double alpha;
  double alpha_dot;
};

STATE state;

// --- PID gains for balancing ---
float Kp_bal = 15;   // proportional gain (tune)12
// float Kd_bal = -150.0;    // derivative gain (tune)
float Kd_bal = 1.35;    // derivative gain (tune)0.85
float Ki_bal = 2;    // optional integral2

float balance_integral = 0.0;

float Kp_vel = 0.2;   // tune0.2
float Ki_vel = 0.08;   // tune0.08
float Kd_vel = 0.000001;   // tune0.00000001
double vel_integral = 0.0;
double vel_prev_error = 0.0;

double freq = 100;

// --- Exponential filters ---
double v_enc_filtered = 0.0;
double theta_filtered = 0.0;

float alpha_vel = 0.8;    // smoothing for velocity
float alpha_theta = 0.6;  // smoothing for pitch angle


void kalmanUpdate(double accel, double dt, double v_enc) {
  x_est = x_est + accel * dt;
  P = P + Q;
  double K = P / (P + R);
  x_est = x_est + K * (v_enc - x_est);
  P = (1 - K) * P;
}
void kalmanUpdateAngular(double gyro_omega, double dt, double omega_enc) {
  omega_est = gyro_omega;
  P_omega = P_omega + Q_omega;
  double K = P_omega / (P_omega + R_omega);
  omega_est = omega_est + K * (omega_enc - omega_est);
  P_omega = (1 - K) * P_omega;
}

void setup() {
  Serial.begin(115200);
  setupJOY();
  motor_driver_setup();  
  setupServoIK();
  delay(1000);
  setupIMU();
}

void loop() {
  double loop_start_time = millis();
  // JOYSTICK
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime;
    joy_cmd = UpdateJoyloop();
    // Serial.println(String(joy_cmd.on_off_state) + " " + 
    //                String(joy_cmd.body_height_cmd) + " " + 
    //                String(joy_cmd.left_joy_cmd) + " " + 
    //                String(joy_cmd.right_joy_cmd));
  }
  if(joy_cmd.on_off_state == -1){
    motor_driver_loop(0.0,0.0);
    ServoIK(0, joy_cmd.body_height_cmd);
    delay(10);
    return;
  }
  // KALMAN
  IMUData imuData = updateIMU();
  DriveVel drive_vel = query_vel();

  double accel = imuData.linearAccX;     // m/s^2
  // double alpha_vel = 0.6;
  double v_enc = alpha_vel * v_enc + (1-alpha_vel) * drive_vel.lin_vel;
  double omega_enc = drive_vel.ang_vel;      // from drivetrain
  double omega_gyro = -imuData.angularVelZ;

  double lin_vel_cmd = -joy_cmd.left_joy_cmd;
  double ang_vel_cmd = joy_cmd.right_joy_cmd;

  if(lin_vel_cmd >= -5 && lin_vel_cmd <= 5){
    lin_vel_cmd = 0;
  }
  if(ang_vel_cmd >= -5 && ang_vel_cmd <= 5){
    ang_vel_cmd = 0;
  }
  lin_vel_cmd /= 512; // 0-> +-1
  ang_vel_cmd /= 512; // 0-> +-1

  // Serial.print("Lin Cmd: ");
  // Serial.print(lin_vel_cmd);
  // Serial.print(" Ang Cmd: ");
  // Serial.print(ang_vel_cmd);

  double v_cmd = lin_vel_cmd * 0.5;   // scale joystick to real speed (e.g., 0.5 m/s max)
  double vel_error = v_cmd - v_enc;

  vel_integral += vel_error * (1.0/freq);   // dt ~0.01s
  vel_integral = constrain(vel_integral, -50, 50); // anti-windup

  double vel_derivative = (vel_error - vel_prev_error) / (1.0/freq);
  vel_prev_error = vel_error;
   if(vel_integral >= 25 ) vel_integral = 25;
  if(vel_integral <= -25) vel_integral =-25;
  Serial.print(" vel_error: ");
  Serial.print(vel_error);
  Serial.print(" vel_integral: ");
  Serial.print(vel_integral);
  Serial.print(" vel_derivative: ");
  Serial.print(vel_derivative);

  double vel_pid_output = Kp_vel * vel_error 
                        + Ki_vel * vel_integral 
                        + Kd_vel * vel_derivative;

  double target_pitch = vel_pid_output;  
  target_pitch = constrain(target_pitch, -10.0, 10.0); 

  Serial.print("  Target pitch: ");
  Serial.print(target_pitch);

  static unsigned long lastKFTime = millis();
  unsigned long now = millis();
  double dt_lin = (now - lastKFTime) / 1000.0;
  lastKFTime = now;
  static unsigned long lastTimeOmega = millis();
  unsigned long currentTimeOmega = millis();
  double dt_omega = (currentTimeOmega - lastTimeOmega) / 1000.0;
  lastTimeOmega = currentTimeOmega;

  // Serial.print(" velocity_encoder: ");
  // Serial.print(v_enc);
  // Serial.print(" angular_encoder: ");
  // Serial.print(omega_enc);

  // kalmanUpdate(accel, dt_lin, v_enc);
  // kalmanUpdateAngular(omega_gyro, dt_omega, omega_enc);

  // state.x_dot = x_est;
  // double alpha_theta = 0.8;
  state.theta = alpha_theta * state.theta + (1-alpha_theta) * imuData.pitch;
  state.theta_dot = imuData.angularVelY;
  // state.alpha = imuData.yaw;
  // state.alpha_dot = omega_est;

  double error = target_pitch - state.theta;
  balance_integral += error * ( 1.0 / freq); // dt=0.01s approx
  double derivative = -state.theta_dot;
  if(balance_integral >= 5 ) balance_integral = 5;
  if(balance_integral <= -5) balance_integral =-5;
  Serial.print(" error: ");
  Serial.print(error );
  Serial.print(" balance_integral: ");
  Serial.print(balance_integral );
  Serial.print(" derivative: ");
  Serial.println(derivative);


  double balance_output = Kp_bal * error + Kd_bal * derivative + Ki_bal * balance_integral;

  double steering_output = ang_vel_cmd * 10; // tune scaling

  double left_pwm = balance_output + steering_output;
  double right_pwm = balance_output - steering_output;

  // Clamp
  left_pwm = constrain(left_pwm, -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);

  // --- Send to motors ---
  motor_driver_loop(left_pwm, right_pwm);


  // Serial.print(" lpwm: ");
  // Serial.print(left_pwm, 4);
  // Serial.print(" rpwn: ");
  // Serial.println(right_pwm, 4);

  ServoIK(0, joy_cmd.body_height_cmd);
  // Serial.println(joy_cmd.body_height_cmd);
  // Serial.print("theta ");
  // Serial.print(state.theta);
  // Serial.print("theta_dot ");
  // Serial.println(state.theta_dot);

  // MOTORS DRIVER
  // motor_driver_loop(final_left, final_right);
  
  delay(10);
  freq = (1000/(millis()-loop_start_time));
  // Serial.println("                                                     "+String(freq));
}