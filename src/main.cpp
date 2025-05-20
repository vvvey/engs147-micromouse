#include "Motors.h"
#include "IR.h"
#include "IMU.h"
#include "Encoder.h"
#include "ArduinoMotorShieldR3.h"
#include "ForwardControl.h"

#define TS 10
unsigned long prev_time_milli = 0;
unsigned long start_time_milli = 0;
unsigned long curr_time_milli = 0;

ForwardControl forward;
float nominal_heading = 0.0;

void setup() {
    Serial.begin(115200);
    IMU_init();
    IR_init();
    rightEnc.begin();
    leftEnc.begin();
    motor_driver.init();

    Serial.println("Hello World");

    forward.init(30.0); // Forward at 30 rad/s
    nominal_heading = IMU_readZ(); // Capture initial heading
    delay(100);
}

void loop() {
    start_time_milli = millis();

    while (millis() - start_time_milli < 15000) {
        curr_time_milli = millis(); 
        if (curr_time_milli - prev_time_milli >= TS) {
            prev_time_milli = curr_time_milli;
 
            // --- Front wall stop condition ---
            float front_dist = IR_getDistance(FRONT_IR);
            if (front_dist < 5) {
                //forward.turnLeft(90.0, 30.0);   // turn left 90 degrees at 15 rad/s
                forward.turnRight(90.0, 30.0);  // turn right 90 degrees at 15 rad/s 
            } 
            Serial.print("IMU Heading: ");
            Serial.println(IMU_readZ(), 2);  // 2 decimal places

 
            // --- Side wall balancing ---
            float left_dist = IR_getDistance(LEFT_IR);
            float right_dist = IR_getDistance(RIGHT_IR); 
            float diff = right_dist - left_dist;

             
            float drift_kp = 7.0;
            if (abs(diff) < 0.1) diff = 0.0;

            float angle_offset = constrain(diff * drift_kp, -15.0, 15.0);
            forward.setReferenceAngle(nominal_heading + angle_offset);

            forward.update();
        }
    }

    motor_driver.setSpeeds(0, 0);
    exit(0);
}

