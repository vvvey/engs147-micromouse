#ifndef CONTROL_H
#define CONTROL_H

class Control { // Abstract base class for all control classes
    public:
        virtual void init() = 0;
        virtual void update() = 0;
        virtual bool isFinished() = 0;
        virtual int getTSMillis() = 0;
    };

#endif