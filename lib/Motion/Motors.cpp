#include "ArduinoMotorShieldR3.h"
#include "Encoder.h"
#include "IMU.h"

#include "Motors.h"
#include <Arduino.h>

// Constructor
Motors::Motors() : rightEnc(2, 360.0), leftEnc(3, 360.0) {}

// Methods
void Motors::begin() {
    md.init();
    rightEnc.begin();
    leftEnc.begin();
    IMU_init();
}

void Motors::forward_straight_begin(float velocity) {
    left_ref_omega = -velocity;
    right_ref_omega = velocity;
    ref_angle = IMU_readZ();
    prev_time_ms = 0.0;
}

void Motors::forward_straight_controller() {
    curr_time_ms = millis();

    // 1. Read current angle from IMU (absolute heading)
    curr_angle = IMU_readZ();

    // 2. Update encoder readings and compute current angular velocities
    leftEnc.update(curr_time_ms);
    left_curr_omega = leftEnc.getOmega();

    rightEnc.update(curr_time_ms);
    right_curr_omega = rightEnc.getOmega();

    // 3. Omega (velocity) errors
    right_omega_err = right_ref_omega - right_curr_omega;
    left_omega_err = left_ref_omega - left_curr_omega;

    // 4. Heading error (ref_angle is heading when straight started)
    angle_err = ref_angle - curr_angle;

    // Normalize angle error to [-180, 180]
    if (angle_err > 180.0) {
        angle_err -= 360.0;
    } else if (angle_err < -180.0) {
        angle_err += 360.0;
    }

    // 5. Controllers
    float angle_kp = 0.2;   // Adjust as needed
    float kp = 0.6;          // Adjust as needed

    float angle_ctrl_voltage = angle_err * angle_kp;

    float left_ctrl_voltage = left_omega_err * kp - angle_ctrl_voltage;
    float right_ctrl_voltage = right_omega_err * kp - angle_ctrl_voltage;

    int left_pwm = voltage_to_pwm(left_ctrl_voltage);
    int right_pwm = voltage_to_pwm(right_ctrl_voltage);

    md.setSpeeds(right_pwm, left_pwm);
    

    // 7. Save previous values (if needed for future use)
    prev_angle = curr_angle;
    prev_time_ms = curr_time_ms;

    // 8. Debug print
    // Serial.println("------");
    // Serial.println("Left omega: " + String(left_curr_omega));
    // Serial.println("Right omega: " + String(right_curr_omega));
    // Serial.println("Angle error: " + String(angle_err));
    // Serial.println("Left ctrl voltage: " + String(left_ctrl_voltage));
    // Serial.println("Right ctrl voltage: " + String(right_ctrl_voltage));
}


void Motors::stop() {
    md.setSpeeds(0, 0);
}

int Motors::voltage_to_pwm(float voltage) {
    float pwm;

    if (voltage < 0) {
        pwm = -46.7743 * exp(-0.1678 * voltage) - 0.0324 * exp(-0.9619 * voltage);
    } else if (voltage > 0) {
        pwm = 53.8983 * exp(0.1413 * voltage) + 0.0517 * exp(0.9216 * voltage);
    } else {
        pwm = 0;
    }

    if (pwm > 400.0) pwm = 400.0;
    if (pwm < -400.0) pwm = -400.0;

    return (int)pwm;
}

void Motors::set_right_motor_voltage(float voltage) {
    int pwm = voltage_to_pwm(voltage);
    md.setM1Speed(pwm);
}

void Motors::set_left_motor_voltage(float voltage) {
    int pwm = voltage_to_pwm(voltage);
    md.setM2Speed(pwm);
}