#include "IMU.h"
#include "Motors.h"
#include <Arduino.h>
#include "RotationControl.h"

RotationControl rot_ctrl;
float compensator90(float e0, float e1, float e2, float v1, float v2);
float compensator180(float e0, float e1, float e2, float v1, float v2);


// Define the RotationControl class
RotationControl::RotationControl() {}

int RotationControl::getTSMillis() {
    return 40;
}


float compensator90(float e0, float e1, float e2, float v1, float v2) {
  float A = 0.110963541666667;     // For e_{k}
  float B = -0.203906250000000 * 1.002;   // For e_{k-1}
  float C = 0.0934635416666667;   // For e_{k-2}
  float D = 1.66666666666667;       // For v_{k-1} 
  float E = -0.666666666666667;     // For v_{k-2}
  float M = A * e0 + B * e1 + C * e2 + D * v1 + E * v2;
  return M;
}

float compensator180(float e0, float e1, float e2, float v1, float v2) {
  float A = 0.0460151187904968;     // For e_{k}
  float B = -0.0845572354211663 * 1.002;   // For e_{k-1}
  float C = 0.0387580993520518;   // For e_{k-2}
  float D = 1.72786177105832;       // For v_{k-1} 
  float E = -0.727861771058315;     // For v_{k-2}
  float V = A * e0 + B * e1 + C * e2 + D * v1 + E * v2;
  return V;
}

void RotationControl::init(float heading_deg, float omega) { // cw positive, ccw negative angle
    stop_motors();
    ref_angle = heading_deg;
    if (ref_angle > 180) ref_angle -= 360;
    if (ref_angle < -180) ref_angle += 360;

    done = false;

    angle_err_0 = 0.0;
    angle_err_1 = 0.0;
    angle_err_2 = 0.0;

    ctrl_0 = 0.0;
    ctrl_1 = 0.0;
    ctrl_2 = 0.0;

    loop_counter = 0;

    curr_angle = IMU_readZ();
    angle_err_0 = ref_angle - curr_angle;
    angle_err_0 = ref_angle - curr_angle;
    if (angle_err_0 > 180.0) angle_err_0 -= 360.0;
    else if (angle_err_0 < -180.0) angle_err_0 += 360.0;

    compensator_id = 0;
    if ((abs(angle_err_0) > 0 && abs(angle_err_0) <= 100)) {
        compensator_id = 0; // 90 degree compensator
    } else {
        compensator_id = 1; // 180 degree 
    }
}

void RotationControl::init() {return;}

void RotationControl::update() {
    // IMU Sensor readings
    curr_angle = IMU_readZ();

    // Normalize angle error to [-180, 180]
    angle_err_0 = ref_angle - curr_angle;
    if (angle_err_0 > 180.0) angle_err_0 -= 360.0;
    else if (angle_err_0 < -180.0) angle_err_0 += 360.0;

    int pwm = 0;
    
    if (compensator_id == 0) {
        ctrl_0 = compensator90(angle_err_0, angle_err_1, angle_err_2, ctrl_1, ctrl_2);
        pwm = voltage_to_pwm90(ctrl_0);
    }
    else if (compensator_id == 1) {
        ctrl_0 = compensator180(angle_err_0, angle_err_1, angle_err_2, ctrl_1, ctrl_2);
        pwm = voltage_to_pwm180(ctrl_0);
    }

    motor_driver.setSpeeds(pwm, pwm);


    AWS = 53.8983 * exp(0.1413 * ctrl_0) + 0.0517 * exp(0.9216 * ctrl_0);
    unsigned long currentTime = millis();

    angle_err_2 = angle_err_1;
    angle_err_1 = angle_err_0;

    ctrl_2 = ctrl_1;
    ctrl_1 = ctrl_0;

    // if (loop_counter < arr_size) {
    //     time[loop_counter] = currentTime;
    //     angle[loop_counter] = curr_angle;
    //     error[loop_counter] = angle_err_0;
    //     controleffort[loop_counter] = ctrl_0;
    // }

    loop_counter++;

    if (abs(angle_err_0) < 3 and abs(angle_err_1) < 3 and abs(angle_err_2) < 3) {
        done = true;
        stop_motors();
    }
}

bool RotationControl::isFinished() {
    return done;
}

void RotationControl::logData() {
    // Serial.println("time, angle, error, control");
    // for (int i = 0; i < arr_size; i++) {
    //     Serial.print(time[i]);
    //     Serial.print(", ");
    //     Serial.print(angle[i]);
    //     Serial.print(", ");
    //     Serial.print(error[i]);
    //     Serial.print(", ");
    //     Serial.println(controleffort[i]);
    // }

}