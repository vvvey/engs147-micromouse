#pragma once
#include "Forward2WallControl.h"
#include "Forward2DisControl.h"
#include "RotationControl.h"
#include "Control.h"
#include <Arduino.h>

class MotionController {
public:
    MotionController();
    void update();
    void logData();
    void fwd_to_wall(float heading, float  dis_mm, float speedx, float speedw) ;
    void fwd_to_dis(int distance_mm);
    void rotate(float angle);
    bool isBusy();

private:
    Control* current_control = nullptr;
    unsigned long last_update_time_ms = 0;
    Forward2WallControl* fwd_wall_ptr = nullptr;
    Forward2DisControl* fwd_dis_ptr = nullptr;
    RotationControl* rot_ptr = nullptr;
};
