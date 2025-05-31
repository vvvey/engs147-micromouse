#ifndef CONTROL_H
#define CONTROL_H

enum ControlState {
    IDLE, // do nothing
    CONSTANT_SPEED, // constant speed mode until a braking distance
    DISTANCE // to target distance
};

class Control { // Abstract base class for all control classes
    public:
        virtual void init() = 0;
        virtual void update() = 0;
        virtual bool isFinished() = 0;
        virtual int getTSMillis() = 0;
        virtual void logData() = 0;
    };

#endif