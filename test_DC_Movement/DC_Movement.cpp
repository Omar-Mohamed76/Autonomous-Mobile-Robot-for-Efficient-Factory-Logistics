#include <Arduino.h>
#include "DC_Movement.h"

// --- Define PWM Properties ---
const int PWM_FREQ = 5000;       // 5 kHz frequency
const int PWM_RESOLUTION = 8;    // 8-bit resolution (0-255)

// --- Constructor ---
// Stores the pins and assigns unique PWM channels
RobotDrivetrain::RobotDrivetrain(int L_lpwm, int L_rpwm, int R_lpwm, int R_rpwm) {
    _L_LPWM_PIN = L_lpwm;
    _L_RPWM_PIN = L_rpwm;
    _R_LPWM_PIN = R_lpwm;
    _R_RPWM_PIN = R_rpwm;

    // Assign PWM channels (0-15 are available on ESP32)
    _L_L_CHANNEL = 0;
    _L_R_CHANNEL = 1;
    _R_L_CHANNEL = 2;
    _R_R_CHANNEL = 3;
}

// --- init() ---
// Sets up the ESP32's ledc channels
void RobotDrivetrain::init() {
    // Setup Left Motor Channels
    ledcSetup(_L_L_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(_L_LPWM_PIN, _L_L_CHANNEL);
    ledcSetup(_L_R_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(_L_RPWM_PIN, _L_R_CHANNEL);

    // Setup Right Motor Channels
    ledcSetup(_R_L_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(_R_LPWM_PIN, _R_L_CHANNEL);
    ledcSetup(_R_R_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(_R_RPWM_PIN, _R_R_CHANNEL);

    // Ensure motors are stopped at boot
    stop();
}

// --- Private Helper Functions ---

// Controls the Left Motor
void RobotDrivetrain::setLeftMotor(int speed) {
    speed = constrain(speed, -255, 255); // Ensure speed is within bounds

    if (speed > 0) { // Forward
        ledcWrite(_L_L_CHANNEL, speed); // Set LPWM
        ledcWrite(_L_R_CHANNEL, 0);     // Clear RPWM
    } else if (speed < 0) { // Reverse
        ledcWrite(_L_L_CHANNEL, 0);     // Clear LPWM
        ledcWrite(_L_R_CHANNEL, abs(speed)); // Set RPWM
    } else { // Stop (Brake)
        ledcWrite(_L_L_CHANNEL, 0);
        ledcWrite(_L_R_CHANNEL, 0);
    }
}

// Controls the Right Motor
void RobotDrivetrain::setRightMotor(int speed) {
    speed = constrain(speed, -255, 255); // Ensure speed is within bounds

    if (speed > 0) { // Forward
        ledcWrite(_R_L_CHANNEL, speed); // Set LPWM
        ledcWrite(_R_R_CHANNEL, 0);     // Clear RPWM
    } else if (speed < 0) { // Reverse
        ledcWrite(_R_L_CHANNEL, 0);     // Clear LPWM
        ledcWrite(_R_R_CHANNEL, abs(speed)); // Set RPWM
    } else { // Stop (Brake)
        ledcWrite(_R_L_CHANNEL, 0);
        ledcWrite(_R_R_CHANNEL, 0);
    }
}


// --- Public Movement Functions ---

void RobotDrivetrain::moveForward(int speed) {
    setLeftMotor(speed);
    setRightMotor(speed);
}

void RobotDrivetrain::moveBackward(int speed) {
    setLeftMotor(-speed);
    setRightMotor(-speed);
}

void RobotDrivetrain::turnLeft(int speed) {
    setLeftMotor(-speed); // Left motor reverse
    setRightMotor(speed); // Right motor forward
}

void RobotDrivetrain::turnRight(int speed) {
    setLeftMotor(speed);  // Left motor forward
    setRightMotor(-speed);// Right motor reverse
}

void RobotDrivetrain::forwardLeft(int speed) {
    setLeftMotor(speed / 2); // Left motor slower
    setRightMotor(speed);
}

void RobotDrivetrain::forwardRight(int speed) {
    setLeftMotor(speed);
    setRightMotor(speed / 2); // Right motor slower
}

void RobotDrivetrain::backwardLeft(int speed) {
    setLeftMotor(-speed);
    setRightMotor(-speed / 2); // Right motor slower
}

void RobotDrivetrain::backwardRight(int speed) {
    setLeftMotor(-speed / 2); // Left motor slower
    setRightMotor(-speed);
}

void RobotDrivetrain::stop() {
    setLeftMotor(0);
    setRightMotor(0);
}