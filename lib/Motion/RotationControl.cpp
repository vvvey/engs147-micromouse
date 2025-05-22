#include "Encoder.h"
#include "IMU.h"
#include "IR.h"
#include "Motors.h"
#include <Arduino.h>
#include <math.h>

#include "RotationControl.h"

// Define the RotationControl class
RotationControl::RotationControl() {}

void RotationControl::init(float angle) {
    return;
}

void RotationControl::init() {
    return;
}

void RotationControl::update() {

}

bool RotationControl::isFinished() {
    return false;
}
