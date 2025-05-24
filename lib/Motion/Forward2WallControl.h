#ifndef FORWARD_2_WALL_CONTROL_H
#define FORWARD_2_WALL_CONTROL_H

#include "Control.h"

enum ControlState {
    IDLE, // do nothing
    ACCELERATE, // accelerate to steady state velocity
    BRAKING // slow down when see wall
};

enum ControlMode {
    EXPLORING, // wheel speed at 20 rad/s
    RACING //  
};

class Forward2WallControl : public Control {
public:
    Forward2WallControl();
    void init() override;     
    void init(float heading, float dis2wall, float omega);  
    void update() override;
    void logData() override;
    bool isFinished() override;
    int getTSMillis() override;

private:
    // heading variables
    float ref_heading = 0.0;
    float prev_heading = 0.0;
    float curr_heading = 0.0;
    float heading_err = 0.0;

    // left wheel
    float l_ref_omega = 0.0;
    float l_omega = 0.0;
    float l_err_0 = 0.0;
    float l_err_1 = 0.0;
    float l_err_2 = 0.0;
    float l_ctrl_0 = 0.0;
    float l_ctrl_1 = 0.0;
    float l_ctrl_2 = 0.0;

    // right wheel
    float r_ref_omega = 0.0;
    float r_omega = 0.0;
    float r_err_0 = 0.0;
    float r_err_1 = 0.0;
    float r_err_2 = 0.0;
    float r_ctrl_0 = 0.0;
    float r_ctrl_1 = 0.0;
    float r_ctrl_2 = 0.0;

    float side_ir_err_0 = 0.0;
    float side_ir_err_1 = 0.0;
    
    float prev_time_ms = 0.0;
    float curr_time_ms = 0.0;

    float side_dis_err_0 = 0.0;


    bool done = false;

    float target_dis_mm = 0.0;

    int loop_counter = 0; // Count how many times update() has run
    float heading_err_1 = 0.0;
    float heading_err_0 = 0.0;
    float heading_ctrl_0 = 0.0; // Heading output correction

    ControlState state;
    ControlMode mode;

    static constexpr int arr_size = 500;
    
    float l_omega_arr[arr_size];
    float r_omega_arr[arr_size];

    float l_ctrl_arr[arr_size];
    float r_ctrl_arr[arr_size];

    long time_ms_arr[arr_size];


};

#endif
