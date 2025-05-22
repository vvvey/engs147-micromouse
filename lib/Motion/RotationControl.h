#ifndef ROTATION_CONTROL_H
#define ROTATION_CONTROL_H

#include "Control.h"

class RotationControl : public Control {
public:
RotationControl();
    void init(float angle); 
    void init() override;      
    void update() override;
    bool isFinished() override;

private:
    // define the variables needed for the RotationControl class
};

#endif
