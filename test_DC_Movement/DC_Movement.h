#include <Arduino.h>

class RobotDrivetrain {
public:
    /**
     * @brief Constructor for the robot drivetrain.
     * @param L_lpwm Pin for Left Motor Forward PWM (LPWM)
     * @param L_rpwm Pin for Left Motor Reverse PWM (RPWM)
     * @param R_lpwm Pin for Right Motor Forward PWM (LPWM)
     * @param R_rpwm Pin for Right Motor Reverse PWM (RPWM)
     */
    RobotDrivetrain(int L_lpwm, int L_rpwm, int R_lpwm, int R_rpwm);

    /**
     * @brief Initializes the ESP32 PWM channels. Call this in your setup().
     */
    void init();

    // --- Main Movement Functions ---
    // Speed is an integer from 0 to 255

    /**
     * @brief (F) Move both motors forward.
     */
    void moveForward(int speed);

    /**
     * @brief (B) Move both motors backward.
     */
    void moveBackward(int speed);

    /**
     * @brief (L) Pivot turn left (left motor reverse, right motor forward).
     */
    void turnLeft(int speed);

    /**
     * @brief (R) Pivot turn right (left motor forward, right motor reverse).
     */
    void turnRight(int speed);

    /**
     * @brief (FL) Veer forward and left (left motor slower than right).
     */
    void forwardLeft(int speed);

    /**
     * @brief (FR) Veer forward and right (right motor slower than left).
     */
    void forwardRight(int speed);

    /**
     * @brief (BL) Veer backward and left (right motor slower than left).
     */
    void backwardLeft(int speed);

    /**
     * @brief (BR) Veer backward and right (left motor slower than right).
     */
    void backwardRight(int speed);

    /**
     * @brief (STOP) Stops both motors (low-side brake).
     */
    void stop();

private:
    // Helper functions to control each motor individually
    // Speed is from -255 (full reverse) to 255 (full forward)
    void setLeftMotor(int speed);
    void setRightMotor(int speed);

    // Pin assignments
    int _L_LPWM_PIN;
    int _L_RPWM_PIN;
    int _R_LPWM_PIN;
    int _R_RPWM_PIN;

    // ESP32 LEDC PWM Channels
    int _L_L_CHANNEL; // Left motor, LPWM
    int _L_R_CHANNEL; // Left motor, RPWM
    int _R_L_CHANNEL; // Right motor, LPWM
    int _R_R_CHANNEL; // Right motor, RPWM
};

