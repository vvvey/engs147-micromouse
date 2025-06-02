#include "TOF.h"
#include <Wire.h>
#include "Adafruit_VL6180X.h"

#define PCA9548A_ADDR 0x70
#define NUM_VL6180_SENSORS 4

Adafruit_VL6180X sensors[NUM_VL6180_SENSORS];

static void selectChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10);  // Allow channel switch to settle
}

void TOF_init() {
  Wire.begin();
  for (uint8_t i = 0; i < NUM_VL6180_SENSORS; i++) {
    selectChannel(i);
    if (!sensors[i].begin()) {
      Serial.print("VL6180 Sensor "); Serial.print(i); Serial.println(" failed to initialize!");
    } else {
      Serial.print("VL6180 Sensor "); Serial.print(i); Serial.println(" initialized.");
    }
  }
}

float TOF_getDistance(uint8_t sensor_id) {
    if (sensor_id >= NUM_VL6180_SENSORS) {
        return -100.0f;
    }

    selectChannel(sensor_id);

    uint8_t range = sensors[sensor_id].readRange();
    uint8_t status = sensors[sensor_id].readRangeStatus();

    if (status == VL6180X_ERROR_NONE) {
        return (float)range;
    }

    delay(2); // small delay to prevent flooding the I2C bus

    return -100.0f; // No valid reading found
}


