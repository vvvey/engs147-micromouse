#include "Encoder.h"
#include "IMU.h"
#include "IR.h"
#include "Motors.h"
#include <Arduino.h>
#include "RotationControl.h"

RotationControl rot_ctrl;
float compensator(float err_0, float err_1, float voltage_1);

// Define the RotationControl class
RotationControl::RotationControl() {}

void RotationControl::init(float heading_deg, float omega) { // cw positive, ccw negative angle
    stop_motors();
    ref_angle = heading_deg;
    if (ref_angle > 180) ref_angle -= 360;
    if (ref_angle < -180) ref_angle += 360;

    leftEnc.reset();
    rightEnc.reset();
    done = false;

    stable_count = 0;
    integral_sum = 0.0;

    Serial.print("ref_angle: ");
    Serial.println(ref_angle);
    Serial.println("------------------");
}

void RotationControl::init() {return;}

void RotationControl::update() {
    // Sensor readings
    curr_angle = IMU_readZ();


    // Update previous values
    ctrl_1 = ctrl_0;
    angle_err_1 = angle_err_0;

    // PI control
    angle_err_0 = ref_angle - curr_angle;
    if (angle_err_0 > 180.0) angle_err_0 -= 360.0;
    else if (angle_err_0 < -180.0) angle_err_0 += 360.0;


    float kp = 0.04;
    float ki = 0.005;
    if (abs(angle_err_0) < 45) {
        integral_sum += angle_err_0*5.0;  // sum over time
    } else {
        integral_sum += angle_err_0/5.0;  // sum over time
    }
    

    // Anti-windup: clamp integral to prevent overflow
    if (integral_sum > 1000) integral_sum = 1000;
    if (integral_sum < -1000) integral_sum = -1000;

    // ctrl_0 = kp * angle_err_0 + ki * integral_sum;

    ctrl_0 = compensator(angle_err_0, angle_err_1, ctrl_1); 
    
    // if (ctrl_0 > 0 && ctrl_0 < 5.2) ctrl_0 = 5.2;
    // if (ctrl_0 < 0 && ctrl_0 > -5.2) ctrl_0 = -5.2;

    set_left_motor_voltage(ctrl_0);
    set_right_motor_voltage(ctrl_0);

    // Check if the rotation is finished
    float delta_angle = angle_err_0 - angle_err_1;
    if (abs(angle_err_0) < 5.0 && abs(delta_angle) < 0.5) {
    stable_count++;
    } else {
        stable_count = 0;
    }

    if (stable_count >= 10) {  // must be stable for 3 cycles
        done = true;
        stop_motors();
        return;
    }

}

bool RotationControl::isFinished() {
    return done;
}

int RotationControl::getTSMillis() {
    return 10;
}


float compensator(float err_0, float err_1, float voltage_1) { // LEAD
    float a = 2.161;
    float b = -1.873;
    float c = 0.4205;

    return a * err_0 + b * err_1 + c * voltage_1;
}