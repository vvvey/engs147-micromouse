#include "Encoder.h"
#include "ArduinoMotorShieldR3.h"
// #include "BNO055.h"
// #include "NAxisMotion.h"
//#include "IR.h"
#include "IMU.h"

Encoder rightEnc(2, 1204.0);  // encoder 1, 1440 ticks/rev


unsigned long prev_time = 0;
const unsigned long TS = 10; // sample period in ms
unsigned long start_time = 0;
const unsigned long RECORD_TIME = 5000;

void setup() {
    Serial.begin(115200);
    IMU_init();
    //IR_init();
    analogReadResolution(12);
    start_time = millis();
}

void loop() {
    unsigned long curr_time = millis();

    if (true) {
        if ((curr_time - prev_time) >= TS) {
            // Encoder Test
            /*rightEnc.update(curr_time);
            float omega = rightEnc.getOmega();  // rad/s

            Serial.println(omega);*/

            // IF Test
            /*float dis = IR_getDistance(FRONT_IR);
            Serial.println(dis);*/

            // IMU Test
            float heading = IMU_readZ();
            Serial.print("Heading: ");
            Serial.print(heading, 2);
            Serial.println(" deg");

            prev_time = curr_time;
        }
    }
}
