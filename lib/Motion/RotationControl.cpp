#include "Encoder.h"
#include "IMU.h"
#include "IR.h"
#include "Motors.h"
#include <Arduino.h>
#include "RotationControl.h"

RotationControl rot_ctrl;
float compensator(float e0, float e1, float e2, float v1, float v2);

// Define the RotationControl class
RotationControl::RotationControl() {}

int RotationControl::getTSMillis() {
    return 40;
}

float compensator(float e0, float e1, float e2, float v1, float v2) {
  float A = 0.110963541666667;     // For e_{k}
  float B = -0.203906250000000;   // For e_{k-1}
  float C = 0.0934635416666667;   // For e_{k-2}
  float D = 1.66666666666667;       // For v_{k-1} 
  float E = -0.666666666666667;     // For v_{k-2}
  float V = A * e0 + B * e1 + C * e2 + D * v1 + E * v2;
  return V;
}

void RotationControl::init(float heading_deg, float omega) { // cw positive, ccw negative angle
    stop_motors();
    ref_angle = heading_deg;
    if (ref_angle > 180) ref_angle -= 360;
    if (ref_angle < -180) ref_angle += 360;

    leftEnc.reset();
    rightEnc.reset();
    done = false;

    angle_err_0 = 0.0;
    angle_err_1 = 0.0;
    angle_err_2 = 0.0;

    ctrl_0 = 0.0;
    ctrl_1 = 0.0;
    ctrl_2 = 0.0;

}

void RotationControl::init() {return;}

void RotationControl::update() {
    // IMU Sensor readings
    curr_angle = IMU_readZ();

    // PI control
    angle_err_0 = ref_angle - curr_angle;
    if (angle_err_0 > 180.0) angle_err_0 -= 360.0;
    else if (angle_err_0 < -180.0) angle_err_0 += 360.0;
    
    ctrl_0 = compensator(angle_err_0, angle_err_1, angle_err_2, ctrl_1, ctrl_2); 

    set_left_motor_voltage(ctrl_0);
    set_right_motor_voltage(ctrl_0);


    // AWS = 53.8983 * exp(0.1413 * ctrl_0) + 0.0517 * exp(0.9216 * ctrl_0);
    // unsigned long currentTime = millis();

    // Serial.print("Time (ms): "); Serial.print(currentTime);
    // Serial.print("Angle: "); Serial.print(curr_angle);
    // Serial.print(" | Error: "); Serial.print(angle_err_0);
    // Serial.print(" | Ctrl: "); Serial.print(ctrl_0);
    // Serial.print(" | PWM: "); Serial.println(AWS);

    angle_err_2 = angle_err_1;
    angle_err_1 = angle_err_0;

    ctrl_2 = ctrl_1;
    ctrl_1 = ctrl_0;

}

bool RotationControl::isFinished() {
    return done;
}

void RotationControl::logData() {
    // 
}