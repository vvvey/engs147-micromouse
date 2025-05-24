#pragma once

#include "Control.h"
#include <Arduino.h>

class MotionController {
public:
    MotionController();
    void update();
    void logData();
    void fwd_to_wall(float heading, float distance, float max_velocity);
    void rotate(float angle);
    bool isBusy();

private:
    Control* current_control = nullptr;
    unsigned long last_update_time_ms = 0;
};
