// Control Feedback Layer
#include "Forward2WallControl.h"
#include "TOF.h"
#include "Encoder.h"
#include "IMU.h"
#include "Motors.h"
#include <Arduino.h>
#include "compensators.h"

Forward2WallControl fwd_2_wall_ctrl; // Allow MotionController to extern

#define TOF_RIGHT 0
#define TOF_FRONT_RIGHT 1
#define TOF_FRONT_LEFT 2
#define TOF_LEFT 3

#define STOP_BTN 32


Forward2WallControl::Forward2WallControl() {
    state = IDLE;
}

void Forward2WallControl::init(float heading_deg, float dis_mm, float omega_rad_s) {
    stop_motors();
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
    loop_counter = 0;

    curr_time_ms = millis();
    prev_time_ms = curr_time_ms;

    l_dis_err_0 = -target_dis_mm;
    r_dis_err_0 = -target_dis_mm;

}

void Forward2WallControl::init() {return;} // ignore this

void Forward2WallControl::update() {
    float front_left_tof = TOF_getDistance(TOF_FRONT_LEFT);
    float front_right_tof = TOF_getDistance(TOF_FRONT_RIGHT);


    if (front_left_tof < 0) front_left_tof = 200.0; 
    if (front_right_tof < 0) front_right_tof = 200.0;

    if (digitalRead(STOP_BTN) == LOW) {
        stop_motors();
        // state = IDLE;
        done = true;
        return;
    }

    switch(state) {
        case IDLE:
            if (front_left_tof > 150.0 && front_right_tof > 150.0) {
                // start moving
                state = ACCELERATE;
                // leftEnc.reset();
                // rightEnc.reset();
            } else {
                state = BRAKING; 
            }

        case ACCELERATE:
            // accelerate to steady state velocity
            if (front_left_tof < 200.0 && front_right_tof < 200.0) {
                state = BRAKING;
                return;
            }

        case BRAKING:
            // slow down when see wall
            if (front_left_tof <= target_dis_mm && front_right_tof <= target_dis_mm) {
                done = true;
                stop_motors();
                // state = IDLE;
                return;
            }
            break;
        }

     // ----------------------FRONT DISTANCE CONTROL_________________
    l_dis_err_1 = l_dis_err_0;
    l_dis_err_0 = front_left_tof - target_dis_mm;
    l_dis_ctrl_0 = front_distance_compensator(l_dis_err_0, l_dis_err_1);


    r_dis_err_1 = r_dis_err_0;
    r_dis_err_0 = front_right_tof - target_dis_mm;
    r_dis_ctrl_0 = front_distance_compensator(r_dis_err_0, r_dis_err_1);


    if (state == BRAKING) {
        l_ref_omega = -l_dis_err_0 * 0.0050f;
        r_ref_omega = r_dis_err_0 * 0.0050f;
    }
     // ----------------------OMEGA CONTROL_________________
    l_omega = leftEnc.getOmega();
    r_omega = rightEnc.getOmega();

    l_err_2 = l_err_1;
    l_err_1 = l_err_0;
    l_err_0 = l_ref_omega - l_omega;

    l_ctrl_2 = l_ctrl_1;
    l_ctrl_1 = l_ctrl_0;
    l_ctrl_0 = omega_compensator(l_err_0, l_err_1, l_err_2, l_ctrl_1, l_ctrl_2);

    
    r_err_2 = r_err_1;
    r_err_1 = r_err_0;
    r_err_0 = r_ref_omega - r_omega;

    r_ctrl_2 = r_ctrl_1;
    r_ctrl_1 = r_ctrl_0;
    r_ctrl_0 = omega_compensator(r_err_0, r_err_1, r_err_2, r_ctrl_1, r_ctrl_2);

   

    // ----------------------SIDE CONTROL____________________

    float alpha = 0.1f; // filter strength: lower = smoother
    // side_tof_err = 0.0f; // persistent filtered value

    float left_tof = TOF_getDistance(TOF_LEFT);
    float right_tof = TOF_getDistance(TOF_RIGHT);

    bool left_tof_valid = (left_tof > 0 && left_tof < 110);
    bool right_tof_valid = (right_tof > 0 && right_tof < 110);

    float offset = -13.0; // mm

    if (left_tof_valid && right_tof_valid) {
        side_tof_err_0 = right_tof - left_tof + offset;
    } else if (right_tof_valid) {
        side_tof_err_0 =  -1.0 * (100.0/2.0 - right_tof - offset/2.0);
    } else if (left_tof_valid) {
        side_tof_err_0 =  100.0/2.0 - left_tof + offset/2.0;  
    } else {
        side_tof_err_0 = 0.0;
    }


    // Low-pass filter
    side_tof_err_1 = side_tof_err;
    side_tof_err = alpha * side_tof_err_0 + (1.0f - alpha) * side_tof_err;

    side_tof_ctrl_0 = side_compensator(side_tof_err, side_tof_err_1);

    
    // ------------------- SUMMING CONTROL____________________
    float left_voltage = 0.0;
    float right_voltage = 0.0;
    
    if (state == ACCELERATE) {
        left_voltage =  l_ctrl_0 + side_tof_ctrl_0;
        right_voltage = r_ctrl_0 + side_tof_ctrl_0;
    } else if (state == BRAKING) {
        left_voltage = l_ctrl_0;
        right_voltage = r_ctrl_0;
    }
    

    float left_pwm = voltage_to_pwm(left_voltage);
    float right_pwm = voltage_to_pwm(right_voltage);

    motor_driver.setSpeeds(right_pwm, left_pwm);


    // Log Data
    if (loop_counter < arr_size) {
        time_ms_arr[loop_counter] = millis();
        l_omega_arr[loop_counter] = l_omega;
        r_omega_arr[loop_counter] = r_omega;
        l_ctrl_arr[loop_counter] = left_voltage;
        r_ctrl_arr[loop_counter] = right_voltage;
        heading_arr[loop_counter] = curr_heading;
        heading_err_arr[loop_counter] = heading_err_0;
        heading_ctrl_arr[loop_counter] = heading_ctrl_0;
    }
    

    loop_counter++;

}

bool Forward2WallControl::isFinished() {
    return done;
}

void Forward2WallControl::logData() {
    Serial.println("time_ms, l_omega, r_omega, l_ctrl, r_ctrl, heading, heading_err, heading_ctrl");
    for (int i = 0; i < loop_counter; i++) {
        Serial.print(time_ms_arr[i]);
        Serial.print(", ");
        Serial.print(l_omega_arr[i]);
        Serial.print(", ");
        Serial.print(r_omega_arr[i]);
        Serial.print(", ");
        Serial.print(l_ctrl_arr[i]);
        Serial.print(", ");
        Serial.print(r_ctrl_arr[i]);
        Serial.print(", ");
        Serial.print(heading_arr[i]);
        Serial.print(", ");
        Serial.print(heading_err_arr[i]);
        Serial.print(", ");
        Serial.println(heading_ctrl_arr[i]);
    }
}

int Forward2WallControl::getTSMillis() {
    return 50;
}



