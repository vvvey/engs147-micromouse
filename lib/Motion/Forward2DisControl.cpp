// Control Feedback Layer
#include "Forward2DisControl.h"
#include "TOF.h"
#include "Encoder.h"
#include "IMU.h"
#include "Motors.h"
#include <Arduino.h>
#include "compensators.h"

Forward2DisControl fwd_2_dis_ctrl; // Allow MotionController to extern

#define TOF_RIGHT 0
#define TOF_FRONT_RIGHT 1
#define TOF_FRONT_LEFT 2
#define TOF_LEFT 3

Forward2DisControl::Forward2DisControl() {}

float Forward2DisControl::speed_compensator_L(float target_speed, float current_speed_L) {
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

float Forward2DisControl::speed_compensator_R(float target_speed, float current_speed_R) {
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

float Forward2DisControl::heading_compensator(float curr_heading) {
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

float Forward2DisControl::side_tof_compensator(float tof_L, float tof_R) {
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


void Forward2DisControl::init() {return;}

void Forward2DisControl::init(int heading, int dis_mm, float spdX) {
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


void Forward2DisControl::update() {
    if (done) return;
    leftEnc.update();
    rightEnc.update();

    float left_speed = leftEnc.getSpeedX();
    float right_speed = -rightEnc.getSpeedX();

    float left_dis = leftEnc.getDis();
    float right_dis = -rightEnc.getDis();

    if (index % 5 == 0) { // Sampling time = TS * 5 = 100ms
        float heading = IMU_readZ();
        float tof_L = TOF_getDistance(TOF_LEFT);
        float tof_R = TOF_getDistance(TOF_RIGHT);

        heading_control = heading_compensator(heading);
        side_control = side_tof_compensator(tof_L, tof_R);
    }

    float vL = 0.0;
    float vR = 0.0;
    float dealth_rec = 1.01;

    float remaining_dis = 0.5f * ((target_dis_mm * dealth_rec - left_dis) + (target_dis_mm * dealth_rec - right_dis));

    // Trapezoidal deceleration: Linearly scale down speed as it nears target
    float min_speed = 50.0;   // mm/s, avoid stalling
    float max_speed = speedX; // original set speed
    float slow_down_distance = 200.0;  // start slowing down within 250 mm

    float scaled_speed = max_speed;
    if (remaining_dis < slow_down_distance) {
        scaled_speed = min_speed + (max_speed - min_speed) * (remaining_dis / slow_down_distance);
    }

    // Use same heading/side control as before
    float base_control_L = speed_compensator_L(scaled_speed + heading_control + side_control, left_speed);
    float base_control_R = speed_compensator_R(scaled_speed - heading_control - side_control, right_speed);

    float pwmL = voltage_to_pwm(base_control_L);
    float pwmR = voltage_to_pwm(-base_control_R);

    motor_driver.setSpeeds(pwmR, pwmL);

    if (abs(remaining_dis) < 5.0) { // If within 10 mm of target distance
        stop_motors();
        done = true;
    }

    index++;
}

bool Forward2DisControl::isFinished() {
    return done;
}

void Forward2DisControl::logData() {
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

int Forward2DisControl::getTSMillis() {
    return 20;
}


