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
    int target_dis_mm = 0; // target distance in mm
    
};

#endif