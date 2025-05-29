#ifndef FORWARD_2_DIS_CONTROL_H
#define FORWARD_2_DIS_CONTROL_H

#include "Control.h"

class Forward2DisControl : public Control {
public:
    Forward2DisControl();
    void init() override;
    void init(int dis_mm);
    void update() override;
    bool isFinished() override;
    void logData() override;
    int getTSMillis() override;

private:
    bool done = false;
    int target_dis_mm = 0;

    // time
    unsigned long curr_time_ms = 0;
    unsigned long prev_time_ms = 0;

    // Omega control 
    float l_omega = 0.0f, r_omega = 0.0f;
    float l_ref_omega = 0.0f, r_ref_omega = 0.0f;
    float l_err_0 = 0.0f, l_err_1 = 0.0f, l_err_2 = 0.0f;
    float r_err_0 = 0.0f, r_err_1 = 0.0f, r_err_2 = 0.0f;
    float l_ctrl_0 = 0.0f, l_ctrl_1 = 0.0f, l_ctrl_2 = 0.0f;
    float r_ctrl_0 = 0.0f, r_ctrl_1 = 0.0f, r_ctrl_2 = 0.0f;

    // Side wall correction
    float side_tof_err = 0.0f;
    float side_tof_err_0 = 0.0f;
    float side_tof_err_1 = 0.0f;
    float side_tof_ctrl_0 = 0.0f;

    // Logging
    static const int arr_size = 1000;
    int loop_counter = 0;
    unsigned long time_ms_arr[arr_size];
    float l_omega_arr[arr_size];
    float r_omega_arr[arr_size];
    float l_ctrl_arr[arr_size];
    float r_ctrl_arr[arr_size];
};

#endif
