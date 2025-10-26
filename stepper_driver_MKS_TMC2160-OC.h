#ifndef MOTOR_DRIVER_LEDC_H
#define MOTOR_DRIVER_LEDC_H

#include <Arduino.h>

// --- Pin definitions ---
#define STEP_PIN1 2
#define DIR_PIN1 4
#define STEP_PIN2 25
#define DIR_PIN2 5

// --- Stepper config ---
const int STEPS_PER_REV = 12800; // 1.8° step, full step
const int PWM_RESOLUTION = 1;    // 8-bit PWM resolution

// --- LEDC channels ---
const int CHANNEL1 = 0; // Motor1
const int CHANNEL2 = 1; // Motor2

// --- Internal state ---
float omega_motor1 = 0; // rad/s
float omega_motor2 = 0; // rad/s

float r = 0.145/2;
float L = 0.41;

struct DriveVel{
    float lin_vel;
    float ang_vel;
};
DriveVel drive_vel;

// --- Initialize motors ---
void motor_driver_init() {
    // Step pins as outputs
    pinMode(STEP_PIN1, OUTPUT);
    pinMode(DIR_PIN1, OUTPUT);
    pinMode(STEP_PIN2, OUTPUT);
    pinMode(DIR_PIN2, OUTPUT);

    digitalWrite(DIR_PIN1, LOW);
    digitalWrite(DIR_PIN2, LOW);

    // Setup LEDC channels
    ledcSetup(CHANNEL1, 100, PWM_RESOLUTION); // initial freq low
    ledcAttachPin(STEP_PIN1, CHANNEL1);

    ledcSetup(CHANNEL2, 100, PWM_RESOLUTION);
    ledcAttachPin(STEP_PIN2, CHANNEL2);

    drive_vel.lin_vel = 0;
    drive_vel.ang_vel = 0;
}

// --- Set motor angular velocities (rad/s) ---
void motor_driver_set_omega(float omega1, float omega2) {
    omega_motor1 = omega1;
    omega_motor2 = omega2;

    // --- Motor1 direction ---
    if (omega1 >= 0) digitalWrite(DIR_PIN1, HIGH);
    else digitalWrite(DIR_PIN1, LOW);

    // --- Motor2 direction ---
    if (omega2 < 0) digitalWrite(DIR_PIN2, HIGH);
    else digitalWrite(DIR_PIN2, LOW);

    // --- Convert omega (rad/s) to LEDC step frequency ---
    // Steps per second = (ω [rad/s]) * (STEPS_PER_REV / 2π)
    unsigned long freq1 = (omega1 == 0) ? 0 : abs(omega1) * STEPS_PER_REV / (2.0 * PI);
    unsigned long freq2 = (omega2 == 0) ? 0 : abs(omega2) * STEPS_PER_REV / (2.0 * PI);

    // Apply frequency to LEDC
    if (freq1 > 0) ledcWriteTone(CHANNEL1, freq1);
    else ledcWrite(CHANNEL1, 0);

    if (freq2 > 0) ledcWriteTone(CHANNEL2, freq2);
    else ledcWrite(CHANNEL2, 0);

    // --- Compute robot linear and angular velocity ---
    drive_vel.lin_vel = r * (omega_motor1 + omega_motor2) / 2.0;
    drive_vel.ang_vel = r * (omega_motor2 - omega_motor1) / L;
}

#endif // MOTOR_DRIVER_LEDC_H