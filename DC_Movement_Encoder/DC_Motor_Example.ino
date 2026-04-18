#include "DC_Motor.h"

void setup() {
    motor_init();     // Setup PWM channels and enable pins
    motor_enable();   // Pull EN pins HIGH — motors are now ready
}


/* --- Example 1: Basic movement sequence --- */
void example_basic_movement() {
    motor_forward(200);     // Both wheels forward at speed 200
    delay(2000);

    motor_turn_left(150);   // Pivot left in place
    delay(500);

    motor_backward(180);    // Both wheels backward
    delay(2000);

    motor_stop();           // Stop
}


/* --- Example 2: Control each wheel independently --- */
void example_independent_wheels() {
    motor_set_left(200);    // Only left wheel moves
    delay(1000);

    motor_set_right(200);   // Now right wheel also moves (both running)
    delay(1000);

    motor_set_left(0);      // Stop left wheel only
    delay(1000);

    motor_stop();           // Stop everything
}


/* --- Example 3: Using motor_set_both for custom speeds --- */
void example_custom_speeds() {
    motor_set_both(200, 150);    // Left faster than right → curves right
    delay(2000);

    motor_set_both(-100, 200);   // Left backward, right forward → sharp left spin
    delay(500);

    motor_stop();
}


/* --- Example 4: ROS2 Twist message (when RPi sends commands) --- */
void example_twist_command(float linear_x, float angular_z) {
    int left_pwm  = 0;
    int right_pwm = 0;

    motor_twist_to_pwm(
        linear_x,      // e.g. 0.5  (m/s forward)
        angular_z,     // e.g. 0.3  (rad/s turn left)
        0.30f,         // wheelbase: 30 cm between wheels
        1.0f,          // max speed: 1.0 m/s = PWM 255
        &left_pwm,
        &right_pwm
    );

    motor_set_both(left_pwm, right_pwm);
}


/* --- Example 5: Safe shutdown --- */
void example_emergency_stop() {
    motor_disable();   // Pulls EN pins LOW — hard disable, even PWM signals are ignored
}


void loop() {
    example_basic_movement();
    delay(3000);
}
