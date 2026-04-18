#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_FREQUENCY   20000
#define PWM_RESOLUTION  8
#define PWM_MAX         255

/* ---------------------------------------------------------------
   CONFIGURATION — Fill these in before calling motor_init()
   
   Each BTS7960 module has 6 signal pins:
   
     LPWM  →  Forward PWM  (connect to any ESP32 GPIO)
     RPWM  →  Reverse PWM  (connect to any ESP32 GPIO)
     L_EN  →  Left Enable  (connect to any ESP32 GPIO)
     R_EN  →  Right Enable (connect to any ESP32 GPIO)
     VCC   →  5V
     GND   →  Common GND with ESP32
     
   You have 2 BTS7960 modules — one for Left motor, one for Right motor.
--------------------------------------------------------------- */

/* Left Motor — BTS7960 Module 1 */
#define LEFT_LPWM_PIN    16
#define LEFT_RPWM_PIN    17
#define LEFT_L_EN_PIN    14
#define LEFT_R_EN_PIN    27

/* Right Motor — BTS7960 Module 2 */
#define RIGHT_LPWM_PIN   18
#define RIGHT_RPWM_PIN   19
#define RIGHT_L_EN_PIN   26
#define RIGHT_R_EN_PIN   25


/* ---------------------------------------------------------------
   FUNCTION LIST
   
   SETUP:
     motor_init()                         — call once in setup()
     motor_enable()                       — turn both motors ON
     motor_disable()                      — turn both motors OFF (safe stop)

   SINGLE WHEEL CONTROL:
     motor_set_left(speed)                — left wheel only,  speed: -255 to +255
     motor_set_right(speed)               — right wheel only, speed: -255 to +255

   COMBINED MOVEMENT:
     motor_set_both(left_speed, right_speed)   — independent speed per wheel
     motor_forward(speed)                      — straight forward
     motor_backward(speed)                     — straight backward
     motor_turn_left(speed)                    — pivot left  (in place)
     motor_turn_right(speed)                   — pivot right (in place)
     motor_curve_forward_left(speed)           — curve forward to the left
     motor_curve_forward_right(speed)          — curve forward to the right
     motor_curve_backward_left(speed)          — curve backward to the left
     motor_curve_backward_right(speed)         — curve backward to the right
     motor_stop()                              — stop both motors

   ROS2 / TELEOPERATION:
     motor_twist_to_pwm(linear_x, angular_z,
                        wheelbase, max_linear_speed,
                        &left_pwm, &right_pwm)  — converts Twist msg to PWM values
--------------------------------------------------------------- */

void motor_init();
void motor_enable();
void motor_disable();

void motor_set_left(int speed);
void motor_set_right(int speed);
void motor_set_both(int left_speed, int right_speed);

void motor_forward(int speed);
void motor_backward(int speed);
void motor_turn_left(int speed);
void motor_turn_right(int speed);
void motor_curve_forward_left(int speed);
void motor_curve_forward_right(int speed);
void motor_curve_backward_left(int speed);
void motor_curve_backward_right(int speed);
void motor_stop();

/*
   motor_twist_to_pwm — converts ROS2 Twist message values to PWM
   
   Parameters:
     linear_x        : forward/backward velocity  (m/s, positive = forward)
     angular_z       : rotation velocity           (rad/s, positive = turn left)
     wheelbase       : distance between wheels     (meters)
     max_linear_speed: max speed your robot reaches at PWM 255 (m/s)
     left_pwm_out    : pointer to store left  PWM result  ← needs pointer (output value)
     right_pwm_out   : pointer to store right PWM result  ← needs pointer (output value)

   Usage example:
     int left_pwm, right_pwm;
     motor_twist_to_pwm(0.5, 0.2, 0.3, 1.0, &left_pwm, &right_pwm);
     motor_set_both(left_pwm, right_pwm);
*/
void motor_twist_to_pwm(float linear_x, float angular_z,
                         float wheelbase, float max_linear_speed,
                         int *left_pwm_out, int *right_pwm_out);

#ifdef __cplusplus
}
#endif

#endif
