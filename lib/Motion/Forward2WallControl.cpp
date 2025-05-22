#include "Forward2WallControl.h"
#include "Encoder.h"
#include "IR.h"
#include "IMU.h"
#include "Motors.h"
#include <Arduino.h>

Forward2WallControl fwd_2_wall_ctrl;
#define RIGHT_IR A10
#define LEFT_IR A8
#define FRONT_IR A9


// Control Feedback Layer
Forward2WallControl::Forward2WallControl() {}

void Forward2WallControl::init(float dis, float omega) {
    dis2wall =  dis;
    stop_motors();
    ref_angle = IMU_readZ();
    leftEnc.reset();
    rightEnc.reset();
    prev_time_ms = millis();
    start_time_ms = prev_time_ms;
    done = false;

    left_ref_omega = omega;
    right_ref_omega = -omega;
}

void Forward2WallControl::init() {return;} // ignore this

void Forward2WallControl::update() {
    if (IR_getDistance(FRONT_IR) < dis2wall) {
        done = true;
        motor_driver.setSpeeds(0, 0);
        return;
    }

    curr_time_ms = millis();

    // Update Sensor Data
    curr_angle = IMU_readZ();

    left_curr_omega = leftEnc.getOmega();
    right_curr_omega = rightEnc.getOmega();

    float left_ir = IR_getDistance(LEFT_IR);
    float right_ir = IR_getDistance(RIGHT_IR);

    if (left_ir > 6.0 && right_ir < 6.0) {
        left_ir = 9.0 - right_ir;
    } else if (right_ir > 6.0 && left_ir < 6.0) {
        right_ir = 9.0 - left_ir;
    } else if (left_ir < 6.0 && right_ir < 6.0) {
        left_ir = 0.0;
        right_ir = 0.0;
    }

    // If left IR is closer to the wall, turn right
    
    float ir_err = right_ir - left_ir;


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
    float angle_kp = 0.1;   
    float omega_kp = 0.3;  
    float ir_kp = 0.2;        

    float left_ctrl_voltage = left_omega_err * omega_kp + angle_err * angle_kp + ir_err * ir_kp;
    float right_ctrl_voltage = right_omega_err * omega_kp + angle_err * angle_kp + ir_err * ir_kp;

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

    Serial.print("dis2wall: ");
    Serial.println(dis2wall);
}

bool Forward2WallControl::isFinished() {
    return done;
}

int Forward2WallControl::getTSMillis() {
    return ts;
}

