// Control Feedback Layer
#include "Forward2WallControl.h"
#include "TOF.h"
#include "Encoder.h"
#include "IMU.h"
#include "Motors.h"
#include <Arduino.h>

Forward2WallControl fwd_2_wall_ctrl; // Allow MotionController to extern

#define TOF_RIGHT 0
#define TOF_FRONT_RIGHT 1
#define TOF_FRONT_LEFT 2
#define TOF_LEFT 3

float fast_omega_compensator(float e0, float e1, float e2, float v1, float v2) {
    // Lead & I Compensator at 10ms sampling rate
    float a = 0.1901;
    float b = - 0.1801;
    float c = 0.0;
    float d = 1.0;
    float e = 0;

    // float a = 0.35;
    // float b = 0.0;
    // float c = 0.0;
    // float d = 0.0;
    // float e = 0.0;
   
    float u = a * e0 + b * e1 + c * e2 + d * v1 + e * v2;
    return u;
}
// Four different compensators
float exploring_acc_gc(float e0, float e1, float e2, float v1, float v2);
float racing_acc_gc(float e0, float e1, float e2, float v1, float v2);
float fast_omega_compensator(float e0, float e1, float v1);
float fast_heading_compensator(float e0, float e1, float omega1);
// float exploring_brake_gc();
// float racing_break_gc();


Forward2WallControl::Forward2WallControl() {
    state = IDLE;
    mode = EXPLORING;
}

void Forward2WallControl::init(float heading_deg, float dis_mm, float omega_rad_s) {
    stop_motors();

    if (omega_rad_s < 30.0) mode = EXPLORING;
    else mode = RACING;

    state = IDLE;

    // set target parameters
    target_dis_mm =  dis_mm;
    ref_heading = heading_deg;
    l_ref_omega = omega_rad_s;
    r_ref_omega = -omega_rad_s;

    // reset 
    leftEnc.reset();
    rightEnc.reset();
    done = false;

    curr_time_ms = millis();
    prev_time_ms = curr_time_ms;

    loop_counter = 0;
    // arr_size = 1000;
}

void Forward2WallControl::init() {return;} // ignore this

void Forward2WallControl::update() {
    

    float fl_dis_mm = TOF_getDistance(TOF_FRONT_LEFT);
    float fr_dis_mm = TOF_getDistance(TOF_FRONT_RIGHT);
    float avg_dis_mm = (fl_dis_mm + fr_dis_mm) / 2.0;
    if (avg_dis_mm < 0.0) avg_dis_mm = 200.0;

    if (avg_dis_mm < 80.0) {
        stop_motors();
        done = true;
        return;
    }
    

    // Outer loop update every 5 calls
    if (loop_counter % 5 == 0) {
        curr_heading = IMU_readZ(); // degrees
        heading_err_1 = heading_err_0;
        heading_err_0 = ref_heading - curr_heading;

        // Normalize angle error to [-180, 180]
        if (heading_err_0 > 180.0) heading_err_0 -= 360.0;
        if (heading_err_0 < -180.0) heading_err_0 += 360.0;

    
        heading_ctrl_0 = fast_heading_compensator(heading_err_0, heading_err_1, heading_ctrl_0);
        heading_ctrl_0 = constrain(heading_ctrl_0, -12.0, 12.0); // in rad/s
    }

    // Adjust omega targets with heading correction

    float heading_ctrl_0 = 0.0;
    
    float l_omega_ref = l_ref_omega + heading_ctrl_0;
    float r_omega_ref = r_ref_omega + heading_ctrl_0;

    // Update motor omega and compute inner loop control
    l_omega = leftEnc.getOmega();
    r_omega = rightEnc.getOmega();

    l_err_2 = l_err_1;
    l_err_1 = l_err_0;
    l_err_0 = l_omega_ref - l_omega;

    r_err_2 = r_err_1;
    r_err_1 = r_err_0;
    r_err_0 = r_omega_ref - r_omega;

    l_ctrl_2 = l_ctrl_1;
    l_ctrl_1 = l_ctrl_0;
    l_ctrl_0 = fast_omega_compensator(l_err_0, l_err_1, l_err_2, l_ctrl_1, l_ctrl_1);

    r_ctrl_2 = r_ctrl_1;
    r_ctrl_1 = r_ctrl_0;
    r_ctrl_0 = fast_omega_compensator(r_err_0, r_err_1, r_err_2, r_ctrl_1, r_ctrl_1);

    int left_pwm = voltage_to_pwm(l_ctrl_0);
    int right_pwm = voltage_to_pwm(r_ctrl_0);

    motor_driver.setSpeeds(right_pwm, left_pwm);

    
    curr_time_ms = millis();
    if (loop_counter < arr_size) {
        l_omega_arr[loop_counter] = l_omega;
        r_omega_arr[loop_counter] = r_omega;
        l_ctrl_arr[loop_counter] = l_ctrl_0;
        r_ctrl_arr[loop_counter] = r_ctrl_0;
        time_ms_arr[loop_counter] = curr_time_ms;
    }

    loop_counter++;
}


void Forward2WallControl::logData() {
    Serial.print("loop_counter: ");
    Serial.println(loop_counter);
    Serial.println("time, left, right, left_ctrl, right_ctrl");
    for (int i = 0; i < arr_size; i++) {
        Serial.print(time_ms_arr[i]);
        Serial.print(", ");
        Serial.print(l_omega_arr[i]);
        Serial.print(", ");
        Serial.print(r_omega_arr[i]);
        Serial.print(", ");
        Serial.print(l_ctrl_arr[i]);
        Serial.print(", ");
        Serial.println(r_ctrl_arr[i]);
        delay(10);
    }

}

bool Forward2WallControl::isFinished() {
    return done;
}

int Forward2WallControl::getTSMillis() {
    return 10; // 50 milli seconds
}



float fast_heading_compensator(float e0, float e1, float omega1) {
    // PI Compensator at 50ms sampling rate
    float a =  0.419;
    float b = -0.4174;
    float c = 1.0;

    float d_omega = a * e0 + b * e1 + c * omega1;
    return d_omega;
}