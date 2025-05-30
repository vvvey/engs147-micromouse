#ifndef COMPENSATORS_H
#define COMPENSATORS_H

float omega_compensator(float e0, float e1, float e2, float v1, float v2);
float side_compensator(float e0, float e1);
float front_distance_compensator(float e0, float e1);
float heading_compensator(float err_k, float err_k_1);

#endif
