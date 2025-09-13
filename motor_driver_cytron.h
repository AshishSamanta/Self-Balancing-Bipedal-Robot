#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// Motor driver pins

#define r_pwm 2
#define r_dir 4
#define l_pwm 18
#define l_dir 5

#define r_enc_a 33
#define r_enc_b 32
#define l_enc_a 12
#define l_enc_b 13

const unsigned long encoder_timeout_us = 100000; // 100 ms = 100,000 us

const int pwmResolution = 8;
const int pwmFreq = 5000;
const int pwmMax = 255;

volatile int right_encoder_counter = 0;
volatile int left_encoder_counter = 0;

double omega_r = 0.0, omega_l = 0.0;
const double counts_per_rev = 134.0;

double time_now_r = 0, old_time_r = 0;
double time_now_l = 0, old_time_l = 0;

double command = 0;
String inputString = "";
bool stringComplete = false;

float r = 0.145;
float L = 0.41;

struct DriveVel{
    float lin_vel;
    float ang_vel;
};

double computeOmega(double now, double prev, int direction) {
  double deltaTime = now - prev;
  if (deltaTime <= 0) return 0;
  double omega = (6.283 / counts_per_rev) * 1000000.0 / deltaTime;
  return direction > 0 ? omega : -omega;
}

void right_encoder_callback() {
  time_now_r = micros();
  int dir = digitalRead(r_enc_b) == HIGH ? 1 : -1;
  right_encoder_counter += dir;
  omega_r = computeOmega(time_now_r, old_time_r, dir);
  old_time_r = time_now_r;
}

void left_encoder_callback() {
  time_now_l = micros();
  int dir = digitalRead(l_enc_b) == HIGH ? 1 : -1;
  left_encoder_counter += dir;
  omega_l = computeOmega(time_now_l, old_time_l, dir);
  old_time_l = time_now_l;
}

void right_motor_cmd(double command) {
  int pwm = constrain((int)abs(command), 0, pwmMax);
  digitalWrite(r_dir, command >= 0 ? HIGH : LOW); // Direction
  ledcWrite(0, pwm); // Speed
}

void left_motor_cmd(double command) {
  int pwm = constrain((int)abs(command), 0, pwmMax);
  digitalWrite(l_dir, command >= 0 ? HIGH : LOW); // Direction
  ledcWrite(1, pwm); // Speed
}

void motor_driver_setup() {
  

  pinMode(r_pwm, OUTPUT);
  pinMode(r_dir, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(l_dir, OUTPUT);

  pinMode(r_enc_b, INPUT); pinMode(l_enc_b, INPUT);

  attachInterrupt(digitalPinToInterrupt(r_enc_a), right_encoder_callback, RISING);
  attachInterrupt(digitalPinToInterrupt(l_enc_a), left_encoder_callback, RISING);

//   ledcAttachChannel(in0, pwmFreq, pwmResolution, 0);
//   ledcAttachChannel(in1, pwmFreq, pwmResolution, 1);
//   ledcAttachChannel(in2, pwmFreq, pwmResolution, 2);
//   ledcAttachChannel(in3, pwmFreq, pwmResolution, 3);

  ledcSetup(0, pwmFreq, pwmResolution);
  ledcAttachPin(r_pwm, 0);

  ledcSetup(1, pwmFreq, pwmResolution);
  ledcAttachPin(l_pwm, 1);

  old_time_r = micros();
  old_time_l = micros();
}

void motor_driver_loop(int cmd_left, int cmd_right) {

  left_motor_cmd(cmd_left);
  right_motor_cmd(cmd_right);
}

DriveVel query_vel(){
    unsigned long now = micros();

    double omega_l_actual = omega_l;
    double omega_r_actual = omega_r;

    if ((now - old_time_l) > encoder_timeout_us) {
        omega_l_actual = 0.0;
    }

    if ((now - old_time_r) > encoder_timeout_us) {
        omega_r_actual = 0.0;
    }

    // Serial.print(" omega_l: ");
    // Serial.print(omega_l);
    // Serial.print(" omega_ r: ");
    // Serial.print(omega_r);

    float lin_vel = (r / 2.0) * (omega_l_actual + omega_r_actual);
    float ang_vel = (r / L) * (omega_r_actual - omega_l_actual);

    return {lin_vel, ang_vel};
}


#endif

