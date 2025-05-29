#include "IMU.h"
#include "Encoder.h"
#include "Motors.h"
#include <Arduino.h>
#include "Forward2DisControl.h"

Forward2DisControl fwd_2_dis_ctrl;

#define TOF_RIGHT 0
#define TOF_FRONT_RIGHT 1
#define TOF_FRONT_LEFT 2
#define TOF_LEFT 3

#define STOP_BTN 32 // Used as stop button

float omega_compensator(float e0, float e1, float e2, float v1, float v2) {
    float a = 0.08359;
    float b = 0.01801;
    float c = - 0.06558;
    float d = 1.026;
    float e = -0.02577;

    return a * e0 + b * e1 + c * e2 + d * v1 + e * v2;
}

float side_compensator(float e0, float e1) {
    float kp = 0.0020;
    float ki = 0.0022;    
    float kd = 0.0022;
    float dt = 0.05;     

    static float integral = 0;
    integral += e0 * dt;
    float derivative = (e0 - e1) / dt;

    return kp * e0 + ki * integral + kd * derivative;
}

float front_distance_compensator(float e0, float e1) {
    float kp = 0.003; 
    float kd = 0.0012;
    float dt = 0.05;     
    
    float derivative = (e0 - e1) / dt;

    return kp * e0 + kd * derivative;
}






Forward2DisControl::Forward2DisControl() {}

int Forward2DisControl::getTSMillis() {
    return 50;
}

void Forward2DisControl::init(int dis_mm) {
    target_dis_mm = dis_mm; // set target distance in mm
    done = false;
    stop_motors();

}
void Forward2DisControl::init() {return;} // Ignoring

void Forward2DisControl::update() {
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

    

}

bool Forward2DisControl::isFinished() {
    return done;
}

void Forward2DisControl::logData() {

}