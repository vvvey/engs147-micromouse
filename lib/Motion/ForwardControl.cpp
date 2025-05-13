#include "ForwardControl.h"
#include "Encoder.h"
#include "IMU.h"
#include "Motors.h"
#include <Arduino.h>
#include <math.h>

// Control Feedback Layer
ForwardControl::ForwardControl() {}

void ForwardControl::init(float omega) {
    ref_angle = IMU_readZ();
    leftEnc.reset();
    rightEnc.reset();
    prev_time_ms = millis();
    start_time_ms = prev_time_ms;
    done = false;

    left_ref_omega = -omega;
    right_ref_omega = omega;
}

void ForwardControl::init() {
    return;
}

void ForwardControl::update() {
    if (done) return;
    unsigned long curr_time_ms = millis();

    left_curr_omega = leftEnc.getOmega();
    right_curr_omega = rightEnc.getOmega();

    float curr_angle = IMU_readZ();
    angle_err = ref_angle - curr_angle;
    if (angle_err > 180.0) angle_err -= 360.0;
    if (angle_err < -180.0) angle_err += 360.0;

    left_omega_err = left_ref_omega - left_curr_omega;
    right_omega_err = right_ref_omega - right_curr_omega;

    float kp = 0.6;
    float angle_kp = 0.2;

    float angle_ctrl_voltage = angle_err * angle_kp;
    float left_ctrl_voltage = left_omega_err * kp - angle_ctrl_voltage;
    float right_ctrl_voltage = right_omega_err * kp - angle_ctrl_voltage;

    set_left_motor_voltage(left_ctrl_voltage);
    set_right_motor_voltage(right_ctrl_voltage);

    prev_time_ms = curr_time_ms;
}

bool ForwardControl::isFinished() {
    return done;
}
