#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "ArduinoMotorShieldR3.h"

extern ArduinoMotorShieldR3 motor_driver;

void stop_motors();
int voltage_to_pwm(float voltage);
int voltage_to_pwm45(float voltage);
int voltage_to_pwm90(float voltage);
int voltage_to_pwm180(float voltage);
int voltage_to_pwm_dis(float voltage);
void set_right_motor_voltage(float voltage);
void set_left_motor_voltage(float voltage);

#endif
