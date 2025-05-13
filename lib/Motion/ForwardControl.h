#ifndef FORWARD_CONTROL_H
#define FORWARD_CONTROL_H

#include "Control.h"

class ForwardControl : public Control {
public:
    ForwardControl();
    void init(float omega); 
    void init() override;      
    void update() override;
    bool isFinished() override;

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

    bool done = true;
};

#endif
