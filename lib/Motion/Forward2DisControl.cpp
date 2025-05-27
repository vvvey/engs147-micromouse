#include "IMU.h"
#include "Encoder.h"
#include "Motors.h"
#include <Arduino.h>
#include "Forward2DisControl.h"

Forward2DisControl fwd_2_dis_ctrl;

// Define the RotationControl class
Forward2DisControl::Forward2DisControl() {}

int Forward2DisControl::getTSMillis() {
    return 50;
}

void Forward2DisControl::init(int dis_mm) {
    target_dis_mm = dis_mm; // set target distance in mm
    done = false;
    stop_motors();

}
void Forward2DisControl::init() {return;}

void Forward2DisControl::update() {
    
}

bool Forward2DisControl::isFinished() {
    return done;
}

void Forward2DisControl::logData() {

}