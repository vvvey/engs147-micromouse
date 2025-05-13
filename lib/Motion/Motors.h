#ifndef MOTORS_H
#define MOTORS_H

#include "ArduinoMotorShieldR3.h"
#include "Encoder.h"
#include "IMU.h"
#include <Arduino.h>

class Motors {
public:
    Motors();
    void begin();
    void forward_straight_begin(float velocity);
    void forward_straight_controller();
    void stop();

private:
    ArduinoMotorShieldR3 md;
    Encoder rightEnc;
    Encoder leftEnc;

    float ref_angle = 0.0;
    float prev_angle = 0.0;
    float curr_angle = 0.0;

    float right_ref_omega = 0.0;
    float left_ref_omega = 0.0;

    float right_prev_omega = 0.0;
    float left_prev_omega = 0.0;

    float right_curr_omega = 0.0;
    float left_curr_omega = 0.0;

    float prev_time_ms = 0.0;
    float curr_time_ms = 0.0;

    float left_omega_err = 0.0;
    float right_omega_err = 0.0;

    float angle_err = 0.0;

    int voltage_to_pwm(float voltage);
    void set_right_motor_voltage(float voltage);
    void set_left_motor_voltage(float voltage);
};

#endif
