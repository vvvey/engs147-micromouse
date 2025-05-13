#include "Encoder.h"
#include <AxisEncoderShield3.h>

Encoder rightEnc(2, 360.0);  
Encoder leftEnc(3, 360.0);

Encoder::Encoder(int id, float ticks_per_rev) : encoder_id(id), ticks_per_rev(ticks_per_rev), init_ticks(0), prev_ticks(0), prev_time_ms(0), omega_rad_s(0) {}

void Encoder::begin() {
    initEncoderShield();
    prev_ticks = getEncoderValue(encoder_id);
    prev_time_ms = millis();
    init_ticks = prev_ticks;
    omega_rad_s = 0;
}

float Encoder::getOmega() {
    long curr_time_ms = millis();
    long curr_ticks = getEncoderValue(encoder_id);
    unsigned long dt_ms = curr_time_ms - prev_time_ms;

    if (dt_ms > 0) {
        float dt_sec = dt_ms / 1000.0;
        float revs = (curr_ticks - prev_ticks) / ticks_per_rev;
        omega_rad_s = (revs * 2.0 * PI) / dt_sec;

        prev_ticks = curr_ticks;
        prev_time_ms = curr_time_ms;
    }

    return omega_rad_s;
}

long Encoder::getPosition() {
    return getEncoderValue(encoder_id);
}

void Encoder::reset() {
    init_ticks = prev_ticks;
    omega_rad_s = 0;
    prev_time_ms = millis();
}