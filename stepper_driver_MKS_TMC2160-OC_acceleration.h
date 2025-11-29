#ifndef MOTOR_DRIVER_LEDC_H
#define MOTOR_DRIVER_LEDC_H

#include <Arduino.h>

// --- Pin definitions ---
#define STEP_PIN1 2
#define DIR_PIN1 4
#define STEP_PIN2 25
#define DIR_PIN2 5

// --- Stepper configuration ---
const int STEPS_PER_REV = 12800; // steps per revolution
const int PWM_RESOLUTION = 1;//could be 8bits but we are only using this for on and off


// --- LEDC channels ---
const int CHANNEL1 = 0;
const int CHANNEL2 = 1; 

// --- Physical parameters ---
const float r = 0.145 / 2.0;  // wheel radius (m)
const float L = 0.41;         // wheel separation (m)

// --- Internal state ---
float omega_motor1 = 0.0;   // rad/s
float omega_motor2 = 0.0;   // rad/s
float alpha_motor1 = 0.0;   // rad/s²
float alpha_motor2 = 0.0;   // rad/s²

const float dt = 0.001;     // 1 ms loop period (1000 Hz)
const float MAX_OMEGA = 20.0; // rad/s limit for safety

struct DriveVel {
    float lin_vel;
    float ang_vel;
};
DriveVel drive_vel;

// --- Initialization ---
void motor_driver_init() {
    pinMode(STEP_PIN1, OUTPUT);
    pinMode(DIR_PIN1, OUTPUT);
    pinMode(STEP_PIN2, OUTPUT);
    pinMode(DIR_PIN2, OUTPUT);

    digitalWrite(DIR_PIN1, LOW);
    digitalWrite(DIR_PIN2, LOW);

    ledcSetup(CHANNEL1, 100, PWM_RESOLUTION);
    ledcAttachPin(STEP_PIN1, CHANNEL1);

    ledcSetup(CHANNEL2, 100, PWM_RESOLUTION);
    ledcAttachPin(STEP_PIN2, CHANNEL2);

    drive_vel.lin_vel = 0;
    drive_vel.ang_vel = 0;
}

// --- Set acceleration commands (rad/s²) ---
void motor_driver_set_accel(float alpha1, float alpha2) {
    alpha_motor1 = alpha1;
    alpha_motor2 = alpha2;
}

// --- Update motor speed based on acceleration (1 kHz loop) ---
void motor_driver_update_from_accel() {
    // Integrate acceleration → angular velocity
    omega_motor1 += alpha_motor1 * dt;
    omega_motor2 += alpha_motor2 * dt;

    // Limit maximum speed
    omega_motor1 = constrain(omega_motor1, -MAX_OMEGA, MAX_OMEGA);
    omega_motor2 = constrain(omega_motor2, -MAX_OMEGA, MAX_OMEGA);
    // Serial.print(" Left omega: ");
    // Serial.print(omega_motor1,5);
    // Serial.print(" ");

    // Direction
    digitalWrite(DIR_PIN1, (omega_motor1 >= 0) ? HIGH : LOW);
    digitalWrite(DIR_PIN2, (omega_motor2 < 0) ? HIGH : LOW);

    // Convert ω → step frequency (Hz)
    unsigned long freq1 = (omega_motor1 == 0) ? 0 : abs(omega_motor1) * STEPS_PER_REV / (2.0f * PI);
    unsigned long freq2 = (omega_motor2 == 0) ? 0 : abs(omega_motor2) * STEPS_PER_REV / (2.0f * PI);

    // Apply frequency
    if (freq1 > 0) ledcWriteTone(CHANNEL1, freq1);
    else ledcWrite(CHANNEL1, 0);

    if (freq2 > 0) ledcWriteTone(CHANNEL2, freq2);
    else ledcWrite(CHANNEL2, 0);

    // Compute drive velocities
    drive_vel.lin_vel = r * (omega_motor1 + omega_motor2) * 0.5;
    drive_vel.ang_vel = r * (omega_motor2 - omega_motor1) / L;
// }

#endif // MOTOR_DRIVER_LEDC_H
