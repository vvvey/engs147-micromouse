#include "Encoder.h"
#include "ArduinoMotorShieldR3.h"
// #include "BNO055.h"
// #include "NAxisMotion.h"

Encoder rightEnc(1, 1440.0);  // encoder 1, 1440 ticks/rev

unsigned long prev_time = 0;
const unsigned long TS = 10; // sample period in ms
unsigned long start_time = 0;
const unsigned long RECORD_TIME = 5000;

void setup() {
    Serial.begin(115200);
    rightEnc.begin();
    start_time = millis();
}

void loop() {
    unsigned long curr_time = millis();

    if ((curr_time - start_time) < RECORD_TIME) {
        if ((curr_time - prev_time) >= TS) {
            rightEnc.update(curr_time);
            float omega = rightEnc.getVelocity();  // rad/s

            Serial.println(omega);
            prev_time = curr_time;
        }
    }
}
