#include "MotionController.h"
#include "Forward2WallControl.h"
#include "RotationControl.h"
#include "Arduino.h"

extern Forward2WallControl fwd_2_wall_ctrl;
extern RotationControl rot_ctrl;

MotionController::MotionController() {}

void MotionController::update() {
    if (!current_control) return;

    unsigned long now = millis();
    if (now - last_update_time_ms >= current_control->getTSMillis()) {
        current_control->update();
        last_update_time_ms = now;

        if (current_control->isFinished()) {
            current_control = nullptr;
        }
    }
}

void MotionController::fwd_2_dis(float distance, float max_omega) {
    fwd_2_wall_ctrl.init(distance, max_omega);
    current_control = &fwd_2_wall_ctrl;
    last_update_time_ms = millis();
}

void MotionController::rotate(float angle) {
    rot_ctrl.init(angle, 30.0);  // assumes omega for now
    current_control = &rot_ctrl;
    last_update_time_ms = millis();
}

bool MotionController::isBusy() {
    return current_control != nullptr;
}
