#ifndef ROTATION_CONTROL_H
#define ROTATION_CONTROL_H

#include "Control.h"

class RotationControl : public Control {
    public:
        RotationControl();
        void init() override;
        void init(float angle, float max_omega);
        bool isFinished() override;
        void update() override;
        int getTSMillis() override;

    private:
        int ts = 10;
        bool done = false;

        float ref_angle = 0.0;
        float prev_angle = 0.0;
        float curr_angle = 0.0;

        float right_ref_omega = 0.0;
        float left_ref_omega = 0.0;
        float right_prev_omega = 0.0;
        float left_prev_omega = 0.0;

        float right_curr_omega = 0.0;
        float left_curr_omega = 0.0;

        unsigned long prev_time_ms = 0;
        unsigned long start_time_ms = 0;
        unsigned long curr_time_ms = 0;
        float left_omega_err = 0.0;
        float right_omega_err = 0.0;
        float angle_err = 0.0;
        float left_ctrl_voltage = 0.0;
        float right_ctrl_voltage = 0.0;
};

#endif
