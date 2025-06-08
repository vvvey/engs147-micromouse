#ifndef FORWARD_CONTROL_H
#define FORWARD_CONTROL_H

#include "Control.h"



class Forward : public Control {
public:
    Forward();
    void init() override;
    void init(int heading, int speedX);
    void update() override;
    bool isFinished() override;
    void logData() override;
    int getTSMillis() override;
    int controlType();
    void stop_next_block(); 
    WallReading_t getWallStatus();

private:
    WallReading_t wall_status; // Wall status, initialized in constructor
    ControlState state; 
    int target_dis_mm;
    bool done = false;
    float speedX;    
    int current_dis_mm = 0.0;

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
    float tof_FL = 0.0;
    float tof_FR = 0.0;
    float side_tof_err0 = 0.0;
    float side_tof_err1 = 0.0;
    float side_tof_err = 0.0;
    float side_tof_ctrl_0 = 0.0;
    float side_tof_ctrl_1 = 0.0;
    float side_integral = 0.0;
    float side_tof_compensator(float tof_L, float tof_R);
    float side_control = 0.0;
};

#endif
