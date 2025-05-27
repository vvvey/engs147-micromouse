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

Forward2WallControl::Forward2WallControl() {}

float Forward2WallControl::speed_compensator_L(float target_speed, float current_speed_L) {
    float a = 0.00327;
    float b = 0.0003831;
    float c = -0.002887;
    float d = 1.499;
    float e = -0.499;

    speedL_err2 = speedL_err1;
    speedL_err1 = speedL_err0; 
    speedL_err0 = target_speed - current_speed_L;

    speedL_ctrl2 = speedL_ctrl1;
    speedL_ctrl1 = speedL_ctrl0;
    speedL_ctrl0 = a * speedL_err0 + b * speedL_err1 + c * speedL_err2 + d * speedL_ctrl1 + e * speedL_ctrl2;

    return speedL_ctrl0;
}

float Forward2WallControl::speed_compensator_R(float target_speed, float current_speed_R) {
    float a = 0.00327;
    float b = 0.0003831;
    float c = -0.002887;
    float d = 1.499;
    float e = -0.499;

    speedR_err2 = speedR_err1;
    speedR_err1 = speedR_err0; 
    speedR_err0 = target_speed - current_speed_R;

    speedR_ctrl2 = speedR_ctrl1;
    speedR_ctrl1 = speedR_ctrl0;
    speedR_ctrl0 = a * speedR_err0 + b * speedR_err1 + c * speedR_err2 + d * speedR_ctrl1 + e * speedR_ctrl2;

    return speedR_ctrl0;
}

float Forward2WallControl::heading_compensator(float curr_heading) {
    float a = 1.2;
    float b = 0;
    float c = 0;

    heading_err1 = heading_err0;
    heading_err0 = target_heading - curr_heading;
    if (heading_err0 > 180.0) heading_err0 -= 360.0;
    if (heading_err0 < -180.0) heading_err0 += 360.0;

    heading_ctrl1 = heading_ctrl0;
    heading_ctrl0 = a * heading_err0 + b * heading_err1 + c * heading_ctrl1;

    return heading_ctrl0;
}

float Forward2WallControl::side_tof_compensator(float tof_L, float tof_R) {
    float alpha = 0.1f;
    float offset = -13.0;

    bool tof_validL = (tof_L > 0 && tof_L < 100);
    bool tof_validR = (tof_R > 0 && tof_R < 100);

    if (tof_validL && tof_validR) {
        side_tof_err0 = tof_R - tof_L + offset;
    } else if (tof_validR) {
        side_tof_err0 = -1.0 * (100.0 / 2.0 - tof_R - offset / 2.0);
    } else if (tof_validL) {
        side_tof_err0 = 100.0 / 2.0 - tof_L + offset / 2.0;
    } else {
        side_tof_err0 = 0.0;
    }

    side_tof_err1 = side_tof_err0;
    side_tof_err0 = alpha * side_tof_err0 + (1.0f - alpha) * side_tof_err0;

    float kp = 0.0022;
    float ki = 0.0024;
    float kd = 0.0024;
    float dt = 0.05;

    side_integral += side_tof_err0 * dt;
    side_integral = constrain(side_integral, -80.0, 80.0);

    side_tof_ctrl_0 = kp * side_tof_err0 + kd * (side_tof_err1 - side_tof_err1) / dt + ki * side_integral;
    return side_tof_ctrl_0;
}

float Forward2WallControl::frontL_tof_compensator(float tof_front_left) {
    if (tof_front_left < 0) tof_front_left = 200.0;

    float a = 0.01;
    float b = 0.0;
    float c = 0.0;

    tof_FL_err1 = tof_FL_err0;
    tof_FL_err0 = tof_front_left - target_dis_mm;
    
    tof_FL_ctrl1 = tof_FL_ctrl0;
    tof_FL_ctrl0 = a * tof_FL_err0 + b * tof_FL_err1 + c * tof_FL_ctrl1;

    return tof_FL_ctrl0;
}

float Forward2WallControl::frontR_tof_compensator(float tof_front_right) {
    if (tof_front_right < 0) tof_front_right = 200.0;

    float a =  0.01;
    float b = 0.0;
    float c = 0.0;

    tof_FR_err1 = tof_FR_err0;
    tof_FR_err0 = tof_front_right - target_dis_mm;
    
    tof_FR_ctrl1 = tof_FR_ctrl0;
    tof_FR_ctrl0 = a * tof_FR_err0 + b * tof_FR_err1 + c * tof_FR_ctrl1;

    return tof_FR_ctrl0;
}


void Forward2WallControl::init() {
return;}

void Forward2WallControl::init(float heading, int dis_mm, float spdX, float spdW) {
    target_heading = heading;
    target_dis_mm = dis_mm;
    speedX = spdX;
    current_dis_mm = 0.0;
    integral_error = 0.0;
    done = false;
    prev_left_mm = 0.0;
    prev_right_mm = 0.0;
    stop_motors();
    index = 0;

    leftEnc.reset();  // Reset left encoder
    rightEnc.reset();  // Reset right encoder
}


void Forward2WallControl::update() {
    if (done) return;

    leftEnc.update();
    rightEnc.update();

    float left_speed = leftEnc.getSpeedX();
    float right_speed = rightEnc.getSpeedX();

    float heading = IMU_readZ();
    float tof_L = TOF_getDistance(TOF_LEFT);
    float tof_R = TOF_getDistance(TOF_RIGHT);
    float tof_FL = TOF_getDistance(TOF_FRONT_LEFT);
    float tof_FR = TOF_getDistance(TOF_FRONT_RIGHT);

    if (tof_FL < 0) tof_FL = 200.0;
    if (tof_FR < 0) tof_FR = 200.0;

    float heading_control = heading_compensator(heading);
    float side_control = side_tof_compensator(tof_L, tof_R);

    float frontL_control = frontL_tof_compensator(tof_FL);
    float frontR_control = frontR_tof_compensator(tof_FR);

    float base_control_L = speed_compensator_L(speedX + heading_control, left_speed);
    float base_control_R = speed_compensator_R(speedX - heading_control, -right_speed);

    float vL = base_control_L + side_control;
    float vR = base_control_R - side_control;

    float tof_RL_diff = tof_R - tof_L;
    float tof_RL_ctrl = tof_RL_diff * 0.001; // Adjust gain as needed

    
    if (tof_FL < 180 && tof_FR < 180) {
        if (frontL_control < 0) frontL_control = frontL_control*5.0;
        if (frontR_control < 0) frontR_control = frontR_control*5.0;
        vL = frontL_control;
        vR = frontR_control;
    }
    

    // Serial.print("frontL: ");
    // Serial.print(tof_FL);
    // Serial.print(" frontR: ");
    // Serial.print(tof_FR);
    // Serial.print(" vL: ");
    // Serial.print(frontL_control);
    // Serial.print(" vR: ");
    // Serial.println(frontR_control);



    float pwmL = voltage_to_pwm(vL);
    float pwmR = voltage_to_pwm(-vR);

    motor_driver.setSpeeds(pwmR, pwmL);

    if (abs(tof_FR - target_dis_mm) < 5 && abs(tof_FL - target_dis_mm) < 5) {
        stop_motors();
        done = true;
    }

    index++;
}



bool Forward2WallControl::isFinished() {
    return done;
}

void Forward2WallControl::logData() {
    Serial.println("Time, Left Speed, Right Speed");
    for (int i = 0; i < index; i++) {
        Serial.print(time[i]);
        Serial.print(", ");
        Serial.print(left_speed[i]);
        Serial.print(", ");
        Serial.println(right_speed[i]);
    }
    Serial.println("End of Data");
    
}

int Forward2WallControl::getTSMillis() {
    return 50;
}


