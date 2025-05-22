#include "IMU.h"

NAxisMotion mySensor;                 

void IMU_init(){
    I2C.begin();
    mySensor.initSensor(0x28);                      // Device address, power and reset
    mySensor.setOperationMode(OPERATION_MODE_NDOF); // Full sensor fusion mode
    mySensor.setUpdateMode(AUTO);
}

float IMU_readZ(){
    float heading = mySensor.readEulerHeading();  // Angle in degrees (0–360)
    return heading;
}