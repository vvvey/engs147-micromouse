#include "Encoder.h"
#include <AxisEncoderShield3.h>

Encoder::Encoder(int id, float ticks_per_rev) : encoder_id(id), ticks_per_rev(ticks_per_rev), prev_ticks(0), prev_time_ms(0), omega_rad_s(0) {}

void Encoder::begin() {
    initEncoderShield();
    prev_ticks = getEncoderValue(encoder_id);
    prev_time_ms = millis();
    init_ticks = prev_ticks;
}

void Encoder::update(unsigned long current_time_ms) {
    long curr_ticks = getEncoderValue(encoder_id);
    unsigned long dt_ms = current_time_ms - prev_time_ms;

    if (dt_ms > 0) {
        float dt_sec = dt_ms / 1000.0;
        float revs = (curr_ticks - prev_ticks - init_ticks) / ticks_per_rev;
        omega_rad_s = (revs * 2.0 * PI) / dt_sec;

        prev_ticks = curr_ticks;
        prev_time_ms = current_time_ms;
    }
}

float Encoder::getVelocity() {
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
