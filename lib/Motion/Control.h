#ifndef CONTROL_H
#define CONTROL_H

enum ControlState {
    IDLE, // do nothing
    CONSTANT_SPEED, // constant speed mode until a braking distance
    DISTANCE // to target distance
};

struct WallReading_t {
    float left_tof = 200.0; // Left TOF sensor reading
    float front_left_tof = 200.0; // Front Left TOF sensor reading
    float front_right_tof = 200.0; // Front Right TOF sensor reading
    float right_tof = 200.0; // Right TOF sensor reading
    float dis_traveled_mm = 0.0;
    bool is_valid = false; 
};
 
class Control { // Abstract base class for all control classes
    public:
        virtual void init() = 0;
        virtual void update() = 0;
        virtual bool isFinished() = 0;
        virtual int getTSMillis() = 0;
        virtual void logData() = 0;
        virtual int controlType() = 0; // Return control type
    };

#endif