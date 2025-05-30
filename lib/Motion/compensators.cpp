#include "Compensators.h"

float omega_compensator(float e0, float e1, float e2, float v1, float v2) {
    float a = 0.08359;
    float b = 0.01801;
    float c = -0.06558;
    float d = 1.026;
    float e = -0.02577;
    return a * e0 + b * e1 + c * e2 + d * v1 + e * v2;
}

float side_compensator(float e0, float e1) {
    float kp = 0.0020;
    float ki = 0.0022;
    float kd = 0.0022;
    float dt = 0.05;
    static float integral = 0;
    integral += e0 * dt;
    float derivative = (e0 - e1) / dt;
    return kp * e0 + ki * integral + kd * derivative;
}

float front_distance_compensator(float e0, float e1) {
    float kp = 0.003;
    float kd = 0.0012;
    float dt = 0.05;
    float derivative = (e0 - e1) / dt;
    return kp * e0 + kd * derivative;
}

float heading_compensator(float err_k, float err_k_1) {
    const float Kp = 0.05f;
    const float Kd = 0.02f;
    return Kp * err_k + Kd * (err_k - err_k_1);
}
