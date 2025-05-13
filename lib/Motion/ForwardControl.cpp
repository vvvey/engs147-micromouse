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

    Serial.println("Forward Control Initialized");
}

void ForwardControl::init() {
    return;
}

void ForwardControl::update() {
    if (done) return;

    curr_time_ms = millis();

    // Update Sensor Data
    curr_angle = IMU_readZ();

    left_curr_omega = leftEnc.getOmega();
    right_curr_omega = rightEnc.getOmega();

    // Error calculation
    left_omega_err = left_ref_omega - left_curr_omega;
    right_omega_err = right_ref_omega - right_curr_omega;
    angle_err = ref_angle - curr_angle;

    // Normalize angle error to [-180, 180]
    if (angle_err > 180.0) {
        angle_err -= 360.0;
    } else if (angle_err < -180.0) {
        angle_err += 360.0;
    }

    // Proportional Control
    float angle_kp = 0.04;   
    float kp = 0.3;          

    float angle_ctrl_voltage = angle_err * angle_kp;

    float left_ctrl_voltage = left_omega_err * kp - angle_ctrl_voltage;
    float right_ctrl_voltage = right_omega_err * kp - angle_ctrl_voltage;

    int left_pwm = voltage_to_pwm(left_ctrl_voltage);
    int right_pwm = voltage_to_pwm(right_ctrl_voltage);

    motor_driver.setSpeeds(right_pwm, left_pwm);
    
    // Save previous values 
    prev_angle = curr_angle;
    prev_time_ms = curr_time_ms;
    left_prev_omega = left_curr_omega;
    right_prev_omega = right_curr_omega;

    // For Debugging
    // Serial.print("Left Omega: ");
    // Serial.println(left_curr_omega);
    // Serial.print(" Left Error: ");
    // Serial.println(left_omega_err);
    // Serial.print(" Left Voltage: ");
    // Serial.println(left_ctrl_voltage);

    // Serial.print(" Right Omega: ");
    // Serial.println(right_curr_omega);
    // Serial.print(" Right Error: ");
    // Serial.println(right_omega_err);
    // Serial.print(" Right Voltage: ");
    // Serial.println(right_ctrl_voltage);
    
    // Serial.print(" Angle: ");
    // Serial.println(curr_angle);
    // Serial.print(" Angle Error: ");
    // Serial.println(angle_err);
    // Serial.println("--------------------");
}

bool ForwardControl::isFinished() {
    return done;
}
