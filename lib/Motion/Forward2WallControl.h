#ifndef FORWARD_2_WALL_CONTROL_H
#define FORWARD_2_WALL_CONTROL_H

#include "Control.h"

class Forward2WallControl : public Control {
public:
    Forward2WallControl();

    void init() override;
    void reset();    
    void init(float heading,int dis_mm, float spdX, float spdW);  
    void update() override;
    void logData() override;
    bool isFinished() override;
    int getTSMillis() override;
private:

    ControlState state; // Control state, initialized in constructor
    
    int target_dis_mm;
    bool done = false;
    float speedX;    
    float current_dis_mm = 0.0;

    float integral_error = 0.0;  
    const float dt = 0.05;       // 50ms in seconds
    
    const float WHEEL_BASE_MM = 81.0; // Adjust based on your robot

    float prev_left_mm = 0.0;
    float prev_right_mm = 0.0;

    static constexpr int arr_size = 500;
    float time[arr_size];
    float left_speed[arr_size];
    float left_ctrl[arr_size];
    float right_speed[arr_size];
    float right_ctrl[arr_size];
    int index = 0;

    float target_heading = 0.0; 
    float heading_err0 = 0.0; 
    float heading_err1 = 0.0;
    float heading_ctrl0 = 0.0;
    float heading_ctrl1 = 0.0;
    float heading_compensator(float curr_heading);

    float heading_control = 0.0;


    // speed compensator variables
    float speedL_err0 = 0.0;
    float speedL_err1 = 0.0;
    float speedL_err2 = 0.0;
    float speedL_ctrl0 = 0.0;
    float speedL_ctrl1 = 0.0;
    float speedL_ctrl2 = 0.0;
    float speed_compensator_L(float target_speed = 0.0, float current_speed_L = 0.0);
    

    // right speed compensator variables
    float speedR_err0 = 0.0;
    float speedR_err1 = 0.0;
    float speedR_err2 = 0.0;
    float speedR_ctrl0 = 0.0;
    float speedR_ctrl1 = 0.0;
    float speedR_ctrl2 = 0.0;
    float speed_compensator_R(float target_speed = 0.0, float current_speed_R = 0.0);

    // side TOF variables
    float side_tof_err0 = 0.0;
    float side_tof_err1 = 0.0;
    float side_tof_err = 0.0;
    float side_tof_ctrl_0 = 0.0;
    float side_tof_ctrl_1 = 0.0;
    float side_integral = 0.0;
    float side_tof_compensator(float tof_L, float tof_R);
    float side_control = 0.0;

    // front left TOF variables
    float tof_FL = 0.0;
    float tof_FL_err0 = 0.0;
    float tof_FL_err1 = 0.0;
    float tof_FL_ctrl0 = 0.0;
    float tof_FL_ctrl1 = 0.0;
    float frontL_tof_compensator(float tof_front_left);
    float tof_FL_integral = 0.0;

    // front right TOF variables
    float tof_FR = 0.0;
    float tof_FR_err0 = 0.0;
    float tof_FR_err1 = 0.0;
    float tof_FR_ctrl0 = 0.0;
    float tof_FR_ctrl1 = 0.0;
    float frontR_tof_compensator(float tof_front_right);
    float tof_FR_integral = 0.0;




};

#endif