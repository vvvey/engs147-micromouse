#include "Encoder.h"
#include "IMU.h"
#include "IR.h"
#include "Motors.h"
#include <Arduino.h>
#include "RotationControl.h"

RotationControl rot_ctrl;

// Define the RotationControl class
RotationControl::RotationControl() {}

void RotationControl::init(float angle, float omega) { // cw positive, ccw negative angle
    stop_motors();
    ref_angle = IMU_readZ() + angle;
    if (ref_angle > 180) ref_angle -= 360;
    if (ref_angle < -180) ref_angle += 360;

    leftEnc.reset();
    rightEnc.reset();
    prev_time_ms = millis();
    start_time_ms = prev_time_ms;
    done = false;

    left_ref_omega = omega;
    right_ref_omega = omega;

    Serial.print("ref_angle: ");
    Serial.println(ref_angle);
    Serial.println("------------------");
}

void RotationControl::init() {return;}

void RotationControl::update() {
    // Sensor readings
    curr_angle = IMU_readZ();
    left_curr_omega = leftEnc.getOmega();
    right_curr_omega = rightEnc.getOmega();

    // Angle error
    angle_err = ref_angle - curr_angle;
    if (angle_err > 180.0) angle_err -= 360.0;
    else if (angle_err < -180.0) angle_err += 360.0;

    // Control gains
    float angle_kp = 0.13;   // proportional gain on angle error
    float omega_kd = 0.06;   // damping gain to balance motors

    // Base control from angle error
    float angle_ctrl_voltage = angle_err * angle_kp;

    // Differential omega term (damping)
    float omega_err = (right_curr_omega + left_curr_omega); // should cancel if symmetric
    float damping_voltage = omega_err * omega_kd;

    // Final voltages
    left_ctrl_voltage = angle_ctrl_voltage - damping_voltage;
    right_ctrl_voltage = angle_ctrl_voltage - damping_voltage;

    set_left_motor_voltage(left_ctrl_voltage);
    set_right_motor_voltage(right_ctrl_voltage);

    // Check if the rotation is finished
    if (abs(angle_err) < 5.0) {
        done = true;
        stop_motors();
        return;
    }

    Serial.print("angle_err: ");
    Serial.print(angle_err);
    Serial.print(" left_omega_err: ");
    Serial.print(left_omega_err);
    Serial.print(" right_omega_err: ");
    Serial.print(right_omega_err);
    Serial.print(" left_ctrl_voltage: ");
    Serial.print(left_ctrl_voltage);
    Serial.print(" right_ctrl_voltage: ");
    Serial.println(right_ctrl_voltage);
    


}

bool RotationControl::isFinished() {
    return done;
}

int RotationControl::getTSMillis() {
    return 10;
}
