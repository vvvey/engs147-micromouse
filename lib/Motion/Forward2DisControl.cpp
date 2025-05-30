#include "Forward2DisControl.h"
#include "TOF.h"
#include "Encoder.h"
#include "IMU.h"
#include "Motors.h"
#include <Arduino.h>
#include "compensators.h"

Forward2DisControl fwd_2_dis_ctrl;

#define TOF_RIGHT 0
#define TOF_FRONT_RIGHT 1
#define TOF_FRONT_LEFT 2
#define TOF_LEFT 3

#define STOP_BTN 32


Forward2DisControl::Forward2DisControl() {}

int Forward2DisControl::getTSMillis() {
    return 50;
}

void Forward2DisControl::init(int dis_mm, int heading) {
    IMU_init();
    stop_motors();

    target_dis_mm =  dis_mm;

    leftEnc.begin();
    rightEnc.begin();

    leftEnc.reset();
    rightEnc.reset();
    done = false;
    
    ref_heading = heading;
    curr_heading = IMU_readZ();

    curr_time_ms = millis();
    prev_time_ms = curr_time_ms;
}

void Forward2DisControl::init() {return;} // Ignoring

void Forward2DisControl::update() {
    if (digitalRead(STOP_BTN) == LOW) {
        Serial.println("STOP button pressed.");
        stop_motors();
        done = true;
        return;
    }

    /****** Front Wall Safety Check ******/
    float front_left_tof = TOF_getDistance(TOF_FRONT_LEFT);
    float front_right_tof = TOF_getDistance(TOF_FRONT_RIGHT);
    if (front_left_tof < 0) front_left_tof = 200.0;
    if (front_right_tof < 0) front_right_tof = 200.0;

    Serial.print("Front TOF L: "); Serial.print(front_left_tof);
    Serial.print("  R: "); Serial.println(front_right_tof);

    if (front_left_tof < 60.0 || front_right_tof < 60.0) {
        Serial.println("Front wall too close. Stopping.");
        stop_motors();
        done = true;
        return;
    }

    leftEnc.update();
    rightEnc.update();

    float avg_distance = 0.5f * (leftEnc.getDis() - rightEnc.getDis()); // Neg needed for backwards encoder values


    Serial.print("Avg Distance: "); Serial.println(avg_distance);

    if (avg_distance >= target_dis_mm) {
        Serial.println("Target distance reached.");
        stop_motors();
        done = true;
        return;
    }

    /****** Speed Control ******/
    float e = target_dis_mm - avg_distance;

    const float Kp = 0.1f;                  // proportional gain
    const float MAX_OMEGA = 25.0f;
    const float MIN_OMEGA = 5.0f;

    float target_omega = Kp * e;
    target_omega = constrain(target_omega, MIN_OMEGA, MAX_OMEGA);

    l_ref_omega = target_omega;
    r_ref_omega = target_omega;

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

    /****** Side TOF Sensors ******/
    float left_tof = TOF_getDistance(TOF_LEFT);
    float right_tof = TOF_getDistance(TOF_RIGHT);
    float offset = -13.0f;

    bool left_valid = (left_tof > 0 && left_tof < 110);
    bool right_valid = (right_tof > 0 && right_tof < 110);

    float side_error = 0.0f;
    if (left_valid && right_valid) {
        side_error = right_tof - left_tof + offset;
    } else if (right_valid) {
        side_error = -1.0f * (50.0f - right_tof - offset / 2.0f);
    } else if (left_valid) {
        side_error = 50.0f - left_tof + offset / 2.0f;
    }

    float alpha = 0.1f;
    side_tof_err_1 = side_tof_err;
    side_tof_err = alpha * side_error + (1.0f - alpha) * side_tof_err;
    side_tof_ctrl_0 = side_compensator(side_tof_err, side_tof_err_1);

    // ----- HEADING CONTROL (IMU) -----
    curr_heading = IMU_readZ(); // degrees

    // Normalize heading error to [-180, 180]
    heading_err_1 = heading_err_0;
    heading_err_0 = curr_heading - ref_heading;
    if (heading_err_0 > 180) heading_err_0 -= 360;
    if (heading_err_0 < -180) heading_err_0 += 360;

    // Apply heading compensator (simple P or PD)
    heading_ctrl_0 = heading_compensator(heading_err_0, heading_err_1);


    float left_voltage = l_ctrl_0 + side_tof_ctrl_0 - heading_ctrl_0;
    float right_voltage = r_ctrl_0 + side_tof_ctrl_0 - heading_ctrl_0;


    float left_pwm = voltage_to_pwm(left_voltage);
    float right_pwm = voltage_to_pwm(right_voltage);

    Serial.print("PWM L: "); Serial.print(left_pwm);
    Serial.print("  R: "); Serial.println(right_pwm);

    motor_driver.setSpeeds(-right_pwm, left_pwm); // invert right to go forward

    /****** Log Data ******/
    if (loop_counter < arr_size) {
        time_ms_arr[loop_counter] = millis();
        l_omega_arr[loop_counter] = l_omega;
        r_omega_arr[loop_counter] = r_omega;
        l_ctrl_arr[loop_counter] = left_voltage;
        r_ctrl_arr[loop_counter] = right_voltage;
    }

    Serial.print("Loop counter: "); Serial.println(loop_counter);
    loop_counter++;
}


bool Forward2DisControl::isFinished() {
    return done;
}

void Forward2DisControl::logData() {
    Serial.println("time_ms, l_omega, r_omega, l_ctrl, r_ctrl");
    for (int i = 0; i < loop_counter; i++) {
        Serial.print(time_ms_arr[i]);
        Serial.print(", ");
        Serial.print(l_omega_arr[i]);
        Serial.print(", ");
        Serial.print(r_omega_arr[i]);
        Serial.print(", ");
        Serial.print(l_ctrl_arr[i]);
        Serial.print(", ");
        Serial.println(r_ctrl_arr[i]);
    }
}
