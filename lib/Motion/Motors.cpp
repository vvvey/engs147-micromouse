#include "ArduinoMotorShieldR3.h"
#include "Motors.h"
#include <Arduino.h>
#include <math.h>

// Hardware Layer
ArduinoMotorShieldR3 motor_driver;

void stop_motors() {
    motor_driver.setSpeeds(0, 0);
}

// Convert voltage to PWM using non-linear motor model
int voltage_to_pwm(float voltage) {
    float pwm;

    if (voltage < -1) {
        pwm = -46.7743 * exp(-0.1678 * voltage) - 0.0324 * exp(-0.9619 * voltage);
    } else if (voltage > 1) {
        pwm = 53.8983 * exp(0.1413 * voltage) + 0.0517 * exp(0.9216 * voltage);
    } else {
        pwm = 0;
    }

    if (pwm > 0 && pwm < 35) pwm = 35;
    if (pwm < 0 && pwm > -35) pwm = -35;

    if (pwm > 400.0) pwm = 400.0;
    if (pwm < -400.0) pwm = -400.0;

    return (int)pwm;
}

int voltage_to_pwm90(float voltage) {
    float pwm;

    if (voltage < 0) {
        pwm = -46.7743 * exp(-0.1678 * voltage) - 0.0324 * exp(-0.9619 * voltage);
    } else if (voltage > 0) {
        pwm = 53.8983 * exp(0.1413 * voltage) + 0.0517 * exp(0.9216 * voltage);
    } else {
        pwm = 0;
    }

    if (pwm > 0 && pwm < 125) pwm = 125;
    if (pwm < 0 && pwm > -125) pwm = -125;

    if (pwm > 400.0) pwm = 400.0;
    if (pwm < -400.0) pwm = -400.0;

    return (int)pwm;
}

int voltage_to_pwm180(float voltage) {
    float pwm;

    if (voltage < -1.5) {
        pwm = -46.7743 * exp(-0.1678 * voltage) - 0.0324 * exp(-0.9619 * voltage);
    } else if (voltage > 1.5) {
        pwm = 53.8983 * exp(0.1413 * voltage) + 0.0517 * exp(0.9216 * voltage);
    } else {
        pwm = 0;
    }

    if (pwm > 0 && pwm < 140) pwm = 140;
    if (pwm < 0 && pwm > -140) pwm = -140;

    if (pwm > 400.0) pwm = 400.0;
    if (pwm < -400.0) pwm = -400.0;

    return (int)pwm;
}


int voltage_to_pwm_dis(float voltage) {
    float pwm;

    if (voltage < -0.15) {
        pwm = -46.7743 * exp(-0.1678 * voltage) - 0.0324 * exp(-0.9619 * voltage);
    } else if (voltage > 0.15) {
        pwm = 53.8983 * exp(0.1413 * voltage) + 0.0517 * exp(0.9216 * voltage);
    } else {
        pwm = 0;
    }

    if (pwm > 0 && pwm < 125) pwm = 125;
    if (pwm < 0 && pwm > -125) pwm = -125;

    if (pwm > 400.0) pwm = 400.0;
    if (pwm < -400.0) pwm = -400.0;

    return (int)pwm;
}

// // Set right motor voltage
// void set_right_motor_voltage(float voltage) {
//     int pwm = voltage_to_pwm(voltage);
//     motor_driver.setM1Speed(pwm);
// }

// // Set left motor voltage
// void set_left_motor_voltage(float voltage) {
//     int pwm = voltage_to_pwm(voltage);
//     motor_driver.setM2Speed(pwm);
// }
