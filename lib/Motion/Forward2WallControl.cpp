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
    float a = 0.001842;
    float b = 8.948e-5;
    float c = -0.001753;
    float d = 1.764;
    float e = -0.7641;

    speedL_err2 = speedL_err1;
    speedL_err1 = speedL_err0; 
    speedL_err0 = target_speed - current_speed_L;

    speedL_ctrl2 = speedL_ctrl1;
    speedL_ctrl1 = speedL_ctrl0;
    speedL_ctrl0 = a * speedL_err0 + b * speedL_err1 + c * speedL_err2 + d * speedL_ctrl1 + e * speedL_ctrl2;

    return speedL_ctrl0;
}

float Forward2WallControl::speed_compensator_R(float target_speed, float current_speed_R) {
    float a = 0.001842;
    float b = 8.948e-5;
    float c = -0.001753;
    float d = 1.764;
    float e = -0.7641;

    speedR_err2 = speedR_err1;
    speedR_err1 = speedR_err0; 
    speedR_err0 = target_speed - current_speed_R;

    speedR_ctrl2 = speedR_ctrl1;
    speedR_ctrl1 = speedR_ctrl0;
    speedR_ctrl0 = a * speedR_err0 + b * speedR_err1 + c * speedR_err2 + d * speedR_ctrl1 + e * speedR_ctrl2;

    return speedR_ctrl0;
}

float Forward2WallControl::heading_compensator(float curr_heading) {
    // PD gains
    float Kp = 3.0;  // proportional gain
    float Kd = 0.0;  // derivative gain (tune this as needed)

    // Compute heading error with wrap-around at ±180°
    heading_err1 = heading_err0;
    heading_err0 = target_heading - curr_heading;
    if (heading_err0 > 180.0) heading_err0 -= 360.0;
    if (heading_err0 < -180.0) heading_err0 += 360.0;

    // PD control law
    float derivative = heading_err0 - heading_err1;
    heading_ctrl0 = Kp * heading_err0 + Kd * derivative;

    return heading_ctrl0;
}

float Forward2WallControl::side_tof_compensator(float tof_L, float tof_R) {
    float alpha = 1.0; // range [0, 1], 1.0 means no filtering
    float offset = -13.0; // found when placed in the middle of two walls

    bool tof_validL = (tof_L > 0 && tof_L < 100); // true if there's wall on the left
    bool tof_validR = (tof_R > 0 && tof_R < 100); // true if there's wall on the right

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
    side_tof_err0 = alpha * side_tof_err0 + (1.0f - alpha) * side_tof_err0; // Low-pass filter

    // PD Controller 
    float kp = 0.18;
    float ki = 0.00;
    float kd = 0.05;
    float dt = 0.1;

    side_integral += side_tof_err0 * dt;
    side_integral = constrain(side_integral, -80.0, 80.0);

    side_tof_ctrl_0 = kp * side_tof_err0 + kd * (side_tof_err1 - side_tof_err1) / dt + ki * side_integral;
    return side_tof_ctrl_0;
}

float Forward2WallControl::frontL_tof_compensator(float tof_front_left) {
    if (tof_front_left < 0) tof_front_left = 200.0;

    // PI controller gains
    float Kp = 0.05;  
    float Ki = 0.1;  
    float T = 0.02;   // Control loop period in seconds

    tof_FL_err0 = tof_front_left - target_dis_mm;

    // Accumulate integral of error
    tof_FL_integral += tof_FL_err0 * T;
    // tof_FL_integral = constrain(tof_FL_integral, -100.0, 100.0); // Limit integral to prevent windup

    // PI controller output
    tof_FL_ctrl0 = Kp * tof_FL_err0 + Ki * tof_FL_integral;

    return constrain(tof_FL_ctrl0, -6.0, 6.0); // Limit output to prevent excessive speed
}

float Forward2WallControl::frontR_tof_compensator(float tof_front_right) {
    if (tof_front_right < 0) tof_front_right = 200.0;

    // PI controller gains
    float Kp = 0.05;
    float Ki = 0.1;
    float T = 0.02;   // Control loop period in seconds

    tof_FR_err0 = tof_front_right - target_dis_mm - 8.0;
    // Accumulate integral of error
    tof_FR_integral += tof_FR_err0 * T;
    // tof_FR_integral = constrain(tof_FR_integral, -100.0, 100.0); // Limit integral to prevent windup
    // PI controller output
    tof_FR_ctrl0 = Kp * tof_FR_err0 + Ki * tof_FR_integral;
    return constrain(tof_FR_ctrl0, -6.0, 6.0); // Limit output to prevent excessive speed
}


void Forward2WallControl::init() {
return;}

void Forward2WallControl::init(float heading, int dis_mm, float spdX, float spdW) {
    // Initialize Set points 
    target_heading = heading;
    target_dis_mm = dis_mm;
    speedX = spdX;

    // Reset variables
    current_dis_mm = 0.0;
    integral_error = 0.0;
    done = false;
    prev_left_mm = 0.0;
    prev_right_mm = 0.0;
    index = 0;
    leftEnc.reset();  // Reset distance
    rightEnc.reset();  // Reset distance

    stop_motors();
    state = CONSTANT_SPEED; 
}


void Forward2WallControl::update() {
    if (done) return;


    leftEnc.update();
    rightEnc.update();

    float left_speed = leftEnc.getSpeedX();
    float right_speed = rightEnc.getSpeedX();


    if (index % 5 == 0) { // Sampling time = TS * 5 = 100ms
        float heading = IMU_readZ();
        float tof_L = TOF_getDistance(TOF_LEFT);
        float tof_R = TOF_getDistance(TOF_RIGHT);

        heading_control = heading_compensator(heading);
        side_control = side_tof_compensator(tof_L, tof_R);
    }

    // checking if the robot should switch to distance control
    if (index % 10 == 0 && state == CONSTANT_SPEED) { // Sampling time = TS * 10 = 200ms
        tof_FL = TOF_getDistance(TOF_FRONT_LEFT);
        tof_FR = TOF_getDistance(TOF_FRONT_RIGHT);
        if (tof_FL < 0) tof_FL = 200.0; // Ensure valid TOF reading
        if (tof_FR < 0) tof_FR = 200.0; // Ensure valid TOF reading

        if (tof_FL < 200 && tof_FR < 200) {
            stop_motors();
            state = DISTANCE; 
            delay(1000); // Wait for 1 second
        }
    }

    float vL = 0.0;
    float vR = 0.0;

    switch (state) {
        case CONSTANT_SPEED: {
            float base_control_L = speed_compensator_L(speedX + heading_control + side_control, left_speed);
            float base_control_R = speed_compensator_R(speedX - heading_control - side_control, -right_speed);

            vL = base_control_L;
            vR = base_control_R;

            float pwmL = voltage_to_pwm(vL);
            float pwmR = voltage_to_pwm(-vR);

            motor_driver.setSpeeds(pwmR, pwmL);
            break;
        }

        case DISTANCE: {
            tof_FL = TOF_getDistance(TOF_FRONT_LEFT);
            tof_FR = TOF_getDistance(TOF_FRONT_RIGHT);

            if (tof_FL < 0) tof_FL = 200.0; // Ensure valid TOF reading
            if (tof_FR < 0) tof_FR = 200.0; // Ensure valid TOF reading

            float frontL_control = frontL_tof_compensator(tof_FL);
            float frontR_control = frontR_tof_compensator(tof_FR);

            vL = frontL_control;
            vR = frontR_control;

            float pwmL = voltage_to_pwm180(vL);
            float pwmR = voltage_to_pwm180(-vR);

            motor_driver.setSpeeds(pwmR, pwmL);
            break;
        }
    }

    if (state == DISTANCE && (tof_FR_err0) < 3 && abs(tof_FL_err0) < 3) {
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
    return 20;
}


