
/****************************C:\Users\alz00\OneDrive\Desktop\final_rcCar\Final_Project\Autonomous-Mobile-Robot-for-Efficient-Factory-Logistics\test_DC_Movement************************************************************************************************************
*****************************************************************************************************************************************
**************************************************    Date     :  11/15/2025			*************************************************
**************************************************    Name     :   Omar Mohamed Hamdy 	*************************************************
**************************************************    Version  :   1.0	            	*************************************************
**************************************************    SWC      :  	test_DC_Movement.ino        *****************************************
*****************************************************************************************************************************************
*****************************************************************************************************************************************
*/

#include "DC_Movement.hpp"

// --- DEFINE YOUR ESP32 PINS ---
// Pick any 4 GPIO pins that support output
// Connect these to the LPWM and RPWM pins on your BTS7960 drivers.

// Left Motor Driver Pins
#define L_MOTOR_LPWM 25 // (L) LPWM
#define L_MOTOR_RPWM 26 // (L) RPWM

// Right Motor Driver Pins
#define R_MOTOR_LPWM 27 // (R) LPWM
#define R_MOTOR_RPWM 32 // (R) RPWM

// Create your robot object using the pins defined above
RobotDrivetrain robot(L_MOTOR_LPWM, L_MOTOR_RPWM, R_MOTOR_LPWM, R_MOTOR_RPWM);

// --- WIRING REMINDER ---
// **IMPORTANT:** You MUST also connect the L_EN and R_EN pins on BOTH
// BTS7960 modules to 3.3V (or 5V, check your module) to enable the driver.
// If they are not connected, the motors will not move.
//
// ESP32 GND -> BTS7960 GND
// ESP32 3.3V -> BTS7960 VCC (for logic)
// (optional) ESP32 3.3V -> BTS7960 L_EN
// (optional) ESP32 3.3V -> BTS7960 R_EN
//
// External Battery + -> BTS7960 B+
// External Battery - -> BTS7960 B-
//
// Motor 1 Terminals -> BTS7960 M+ and M-
// (Repeat for second motor and driver)

void setup()
{
    Serial.begin(115200);
    Serial.println("Robot Initializing...");

    // Initialize the motor driver
    robot.init();
}

void loop()
{
    int moveSpeed = 200; // Speed from 0-255
    int turnSpeed = 150;

    Serial.println("Moving Forward (F)");
    robot.moveForward(moveSpeed);
    delay(2000);
    robot.stop();
    delay(1000);

    Serial.println("Moving Backward (B)");
    robot.moveBackward(moveSpeed);
    delay(2000);
    robot.stop();
    delay(1000);

    Serial.println("Turning Right (R)");
    robot.turnRight(turnSpeed);
    delay(1000);
    robot.stop();
    delay(1000);

    Serial.println("Turning Left (L)");
    robot.turnLeft(turnSpeed);
    delay(1000);
    robot.stop();
    delay(1000);

    Serial.println("Veering Forward-Right (FR)");
    robot.forwardRight(moveSpeed);
    delay(1500);
    robot.stop();
    delay(1000);

    Serial.println("Veering Forward-Left (FL)");
    robot.forwardLeft(moveSpeed);
    delay(1500);
    robot.stop();
    delay(1000);

    Serial.println("Veering Backward-Right (BR)");
    robot.backwardRight(moveSpeed);
    delay(1500);
    robot.stop();
    delay(1000);

    Serial.println("Veering Backward-Left (BL)");
    robot.backwardLeft(moveSpeed);
    delay(1500);
    robot.stop();
    delay(1000);

    Serial.println("--- Loop Repeating ---");
    delay(2000);
}