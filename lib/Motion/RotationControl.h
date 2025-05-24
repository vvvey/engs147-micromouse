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
        bool done = false;

        float ref_angle;
        float AWS;
        float prev_angle = 0.0;
        float curr_angle = 0.0;

        float angle_err_0 = 0.0;
        float angle_err_1 = 0.0;
        float angle_err_2 = 0.0;

        float ctrl_0 = 0.0;
        float ctrl_1 = 0.0;
        float ctrl_2 = 0.0;

        int stable_count = 0;
        float integral_sum = 0.0;
};

#endif
