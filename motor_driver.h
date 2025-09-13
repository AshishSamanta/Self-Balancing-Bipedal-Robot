#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

#define in0 2
#define in1 4
#define in2 18
#define in3 5
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
  if(command >= 0){
    ledcWrite(0, pwm);  // in0
    ledcWrite(1, 0);    // in1
  } else {
    ledcWrite(0, 0);    // in0
    ledcWrite(1, pwm);  // in1
  }
}

void left_motor_cmd(double command) {
  int pwm = constrain((int)abs(command), 0, pwmMax);
  if(command >= 0){
    ledcWrite(2, pwm);  // in2
    ledcWrite(3, 0);    // in3
  } else {
    ledcWrite(2, 0);    // in2
    ledcWrite(3, pwm);  // in3
  }
}

void motor_driver_setup() {
  

  pinMode(in0, OUTPUT); pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); pinMode(in3, OUTPUT);

  pinMode(r_enc_b, INPUT); pinMode(l_enc_b, INPUT);

  attachInterrupt(digitalPinToInterrupt(r_enc_a), right_encoder_callback, RISING);
  attachInterrupt(digitalPinToInterrupt(l_enc_a), left_encoder_callback, RISING);

  ledcSetup(0, 5000, 8);
  ledcAttachPin(in0, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(in1, 1);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(in2, 2);
  ledcSetup(3, 5000, 8);
  ledcAttachPin(in3, 3);

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

    // if ((now - old_time_l) > encoder_timeout_us) {
    //     omega_l_actual = 0.0;
    // }

    // if ((now - old_time_r) > encoder_timeout_us) {
    //     omega_r_actual = 0.0;
    // }
    Serial.print(" omega_l: ");
    Serial.print(omega_l);
    Serial.print(" omega_ r: ");
    Serial.print(omega_r);

    float lin_vel = (r / 2.0) * (omega_l_actual + omega_r_actual);
    float ang_vel = (r / L) * (omega_r_actual - omega_l_actual);

    return {lin_vel, ang_vel};
}


#endif

