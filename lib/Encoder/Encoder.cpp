#include "Encoder.h"
#include <AxisEncoderShield3.h>
#include <Arduino.h>

#define WHEEL_RADIUS_MM 15 // Wheel radius in mm

Encoder rightEnc(2, 360.0);  
Encoder leftEnc(3, 360.0);

Encoder::Encoder(int id, float ticks_per_rev) {
    this->encoder_id = id;
    this->ticks_per_rev = ticks_per_rev;
    this->omega = 0.0;
    this->delta_mm = 0.0;
    this->num_rev = 0.0;
    this->total_mm = 0.0;
}

void Encoder::begin() {
    initEncoderShield();
    prev_ticks = getEncoderValue(encoder_id);
    prev_ms = millis(); 
    init_ticks = prev_ticks;
    omega = 0.0;
    num_rev = 0.0;
    delta_mm = 0.0;
}

void Encoder::update() {
    long curr_ms = millis();
    long curr_ticks = getEncoderValue(encoder_id);
    unsigned long dt_ms = curr_ms - prev_ms;

    if (dt_ms > 0) {
        float dt_sec = dt_ms / 1000.0;
        num_rev = (prev_ticks - curr_ticks) / ticks_per_rev;
        delta_mm = num_rev * WHEEL_RADIUS_MM * 2.0 * PI; // Convert revolutions to mm
        total_mm += delta_mm;
        omega_rad_s = (num_rev * 2.0 * PI) / dt_sec;

        prev_ticks = curr_ticks;
        prev_ms = curr_ms;
    }
}

float Encoder::getOmega() {
    return this->omega;
}

float Encoder::getDis() {
    return this->total_mm;
}

float Encoder::getDeltaMM() {
    return this->delta_mm;
}

void Encoder::reset() {
    init_ticks = getEncoderValue(encoder_id);
    omega_rad_s = 0;
    prev_ms = millis();
    prev_ticks = init_ticks;
    num_rev = 0.0;
    delta_mm = 0.0;
    total_mm = 0.0;
}