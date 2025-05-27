#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(int id, float ticks_per_rev);
    void begin();
    void update();
    float getOmega();
    float getDis();
    float getDeltaMM();
    void reset();

private:
    float omega;
    float num_rev;
    float delta_mm;
    float total_mm;
    int encoder_id;
    float ticks_per_rev;
    long init_ticks;
    long prev_ticks;
    unsigned long prev_ms;
    float omega_rad_s;
};

extern Encoder rightEnc;
extern Encoder leftEnc;

#endif
