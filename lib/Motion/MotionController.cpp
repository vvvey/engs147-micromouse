#include "MotionController.h"
#include "Forward2WallControl.h"
#include "Forward2DisControl.h"
#include "RotationControl.h"
#include "Forward.h"
#include "Arduino.h"

MotionController::MotionController() {
    delete fwd_wall_ptr;
    delete fwd_dis_ptr;
    delete rot_ptr;
    delete fwd_ptr;
}

void MotionController::update() {
    if (!current_control) return;

    unsigned long now = millis();
    if (now - last_update_time_ms >= current_control->getTSMillis()) {
        current_control->update();
        last_update_time_ms = now;

        if (current_control->isFinished()) {
            // while (digitalRead(30) == HIGH) { // log button
            //     delay(10);
            // }
            // current_control->logData();
            current_control = nullptr;
            
        }
    }
}

void MotionController::logData() {
    if (!current_control) {
        Serial.println("No control active");
        return;
    }

    if (!current_control->isFinished()) {
        Serial.println("Control is still active");
        return;
    }

    current_control->logData();
}

void MotionController::fwd_to_wall(float heading, float  dis_mm, float speedx, float speedw) {
    // Reset fwd_2_wall_ctrl by creating a new instance and replacing the old one
    if (fwd_wall_ptr) {
        delete fwd_wall_ptr; // Clean up old instance
    }

    fwd_wall_ptr = new Forward2WallControl(); // Allocate a new one
    fwd_wall_ptr->init(heading, dis_mm, speedx, speedw);
    current_control = fwd_wall_ptr;

    last_update_time_ms = millis();
}

void MotionController::fwd_to_dis(int heading, int distance_mm,  float speedx) {
    // Reset fwd_2_dis_ctrl by creating a new instance and replacing the old one
    if (fwd_dis_ptr) {
        delete fwd_dis_ptr; // Clean up old instance
    }
    fwd_dis_ptr = new Forward2DisControl(); // Allocate a new one
    fwd_dis_ptr->init(heading, distance_mm, speedx);
    current_control = fwd_dis_ptr;
    last_update_time_ms = millis();
}

void MotionController::fwd(int heading, int speedx) {
    // Reset fwd_ptr by creating a new instance and replacing the old one
    if (fwd_ptr) {
        delete fwd_ptr; // Clean up old instance
    }
    fwd_ptr = new Forward(); // Allocate a new one
    fwd_ptr->init(heading, speedx);
    current_control = fwd_ptr;
    last_update_time_ms = millis();
}

void MotionController::rotate(float angle) {
    // RotationControl rot_ctrl;
    // Reset rot_ctrl by creating a new instance and replacing the old one
    if (rot_ptr) {
        delete rot_ptr; // Clean up old instance
    }
    rot_ptr = new RotationControl();
    rot_ptr->init(angle, 30.0);
    current_control = rot_ptr;
    last_update_time_ms = millis();
}

bool MotionController::isBusy() {
    return current_control != nullptr;
}

WallReading_t MotionController::getWallStatus() {
    if (!current_control) {
        return WallReading_t(); // Return default if no control is active
    }

    if (current_control->controlType() != 1) { // Forward control
        return WallReading_t();
    }

    if (current_control) {
        return static_cast<Forward*>(current_control)->getWallStatus();
    } else if (fwd_wall_ptr) {
        return fwd_wall_ptr->getWallStatus();
    }
    return WallReading_t(); // Return default if no control is active
}

void MotionController::stop_next_block() {
    if (current_control && current_control->controlType() == 1) { // Forward control
        static_cast<Forward*>(current_control)->stop_next_block();
    } else if (fwd_wall_ptr) {
        fwd_wall_ptr->stop_next_block();
    }
}

int MotionController::controlType() {
    return current_control ? current_control->controlType() : -1; // Return -1 if no control is active
}