#ifndef FORWARD_2_DIS_CONTROL_H
#define FORWARD_2_DIS_CONTROL_H

#include "Control.h"

class Forward2DisControl : public Control {
public:
    Forward2DisControl();
    void init() override;
    void init(float distance, float omega);
    void update() override;
    bool isFinished() override;
    int getTSMillis() override;

private:
    unsigned long prev_time_ms = 0;
    unsigned long start_time_ms = 0;
    float ref_angle = 0.0;
    float prev_angle = 0.0;
    float curr_angle = 0.0;

    float right_ref_omega = 0.0;
    float left_ref_omega = 0.0;

    float right_prev_omega = 0.0;
    float left_prev_omega = 0.0;

    float right_curr_omega = 0.0;
    float left_curr_omega = 0.0;

    unsigned long curr_time_ms = 0;

    float left_omega_err = 0.0;
    float right_omega_err = 0.0;

    float angle_err = 0.0;

    bool done = false;
    int ts = 10;

    float dis2wall = 0.0;
};

#endif
