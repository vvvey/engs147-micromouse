#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(int id, float ticks_per_rev);
    void begin();
    float getOmega();
    long getPosition();
    void reset();

private:
    int encoder_id;
    float ticks_per_rev;
    long init_ticks;
    long prev_ticks;
    unsigned long prev_time_ms;
    float omega_rad_s;
};

extern Encoder rightEnc;
extern Encoder leftEnc;

#endif
