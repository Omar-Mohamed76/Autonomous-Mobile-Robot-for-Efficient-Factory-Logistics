#include "DC_Motor.h"

// mario if you want to use this file with micro ROS with C language, just edit the name to be { DC_Motor.c }
// and in the DC_Moto.h delete:
/* 
#ifdef __cplusplus
extern "C" {
#endif
.
.
.

#ifdef __cplusplus
}
#endif
*/

/* ===============================================================
   SETUP FUNCTIONS
   =============================================================== */

void motor_init() {
    ledcAttach(LEFT_LPWM_PIN,  PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(LEFT_RPWM_PIN,  PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(RIGHT_LPWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(RIGHT_RPWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

    pinMode(LEFT_L_EN_PIN,  OUTPUT);
    pinMode(LEFT_R_EN_PIN,  OUTPUT);
    pinMode(RIGHT_L_EN_PIN, OUTPUT);
    pinMode(RIGHT_R_EN_PIN, OUTPUT);

    motor_disable();
}

void motor_enable() {
    digitalWrite(LEFT_L_EN_PIN,  HIGH);
    digitalWrite(LEFT_R_EN_PIN,  HIGH);
    digitalWrite(RIGHT_L_EN_PIN, HIGH);
    digitalWrite(RIGHT_R_EN_PIN, HIGH);
}

void motor_disable() {
    digitalWrite(LEFT_L_EN_PIN,  LOW);
    digitalWrite(LEFT_R_EN_PIN,  LOW);
    digitalWrite(RIGHT_L_EN_PIN, LOW);
    digitalWrite(RIGHT_R_EN_PIN, LOW);

    ledcWrite(LEFT_LPWM_PIN,  0);
    ledcWrite(LEFT_RPWM_PIN,  0);
    ledcWrite(RIGHT_LPWM_PIN, 0);
    ledcWrite(RIGHT_RPWM_PIN, 0);
}


/* ===============================================================
   SINGLE WHEEL CONTROL
   speed range: -255 (full reverse) to +255 (full forward), 0 = stop
   =============================================================== */

void motor_set_left(int speed) {
    if (speed > PWM_MAX)  speed =  PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;

    if (speed > 0) {
        ledcWrite(LEFT_LPWM_PIN, speed);
        ledcWrite(LEFT_RPWM_PIN, 0);
    } else if (speed < 0) {
        ledcWrite(LEFT_LPWM_PIN, 0);
        ledcWrite(LEFT_RPWM_PIN, -speed);
    } else {
        ledcWrite(LEFT_LPWM_PIN, 0);
        ledcWrite(LEFT_RPWM_PIN, 0);
    }
}

void motor_set_right(int speed) {
    if (speed > PWM_MAX)  speed =  PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;

    if (speed > 0) {
        ledcWrite(RIGHT_LPWM_PIN, speed);
        ledcWrite(RIGHT_RPWM_PIN, 0);
    } else if (speed < 0) {
        ledcWrite(RIGHT_LPWM_PIN, 0);
        ledcWrite(RIGHT_RPWM_PIN, -speed);
    } else {
        ledcWrite(RIGHT_LPWM_PIN, 0);
        ledcWrite(RIGHT_RPWM_PIN, 0);
    }
}


/* ===============================================================
   COMBINED MOVEMENT FUNCTIONS
   =============================================================== */

void motor_set_both(int left_speed, int right_speed) {
    motor_set_left(left_speed);
    motor_set_right(right_speed);
}

void motor_forward(int speed) {
    motor_set_both(speed, speed);
}

void motor_backward(int speed) {
    motor_set_both(-speed, -speed);
}

void motor_turn_left(int speed) {
    motor_set_both(-speed, speed);
}

void motor_turn_right(int speed) {
    motor_set_both(speed, -speed);
}

void motor_curve_forward_left(int speed) {
    motor_set_both(speed / 2, speed);
}

void motor_curve_forward_right(int speed) {
    motor_set_both(speed, speed / 2);
}

void motor_curve_backward_left(int speed) {
    motor_set_both(-speed, -speed / 2);
}

void motor_curve_backward_right(int speed) {
    motor_set_both(-speed / 2, -speed);
}

void motor_stop() {
    motor_set_both(0, 0);
}


/* ===============================================================
   ROS2 / TELEOPERATION
   =============================================================== */

void motor_twist_to_pwm(float linear_x, float angular_z,
                         float wheelbase, float max_linear_speed,
                         int *left_pwm_out, int *right_pwm_out)
{
    float v_left  = linear_x - (angular_z * wheelbase / 2.0f);
    float v_right = linear_x + (angular_z * wheelbase / 2.0f);

    int left_pwm  = (int)((v_left  / max_linear_speed) * 255.0f);
    int right_pwm = (int)((v_right / max_linear_speed) * 255.0f);

    if (left_pwm  >  PWM_MAX) left_pwm  =  PWM_MAX;
    if (left_pwm  < -PWM_MAX) left_pwm  = -PWM_MAX;
    if (right_pwm >  PWM_MAX) right_pwm =  PWM_MAX;
    if (right_pwm < -PWM_MAX) right_pwm = -PWM_MAX;

    *left_pwm_out  = left_pwm;
    *right_pwm_out = right_pwm;
}
