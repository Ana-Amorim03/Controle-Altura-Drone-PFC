#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// MPU-6050: angulos de pitch e roll (graus), yaw integrado (graus, com deriva)
// e velocidades angulares p, q, r (rad/s).
extern float angle_pitch_output, angle_roll_output;
extern float angle_yaw;
extern float rate_p, rate_q, rate_r;

void IMU_setup();
void IMU_update();

#endif
