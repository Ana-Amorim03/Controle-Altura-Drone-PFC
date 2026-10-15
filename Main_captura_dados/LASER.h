#ifndef LASER_H
#define LASER_H

#include <Arduino.h>

// Sensor de distancia VL53L0X (biblioteca VL53L0X da Pololu), no mesmo
// barramento I2C da IMU. Chamar IMU_setup() antes de LASER_setup(), pois
// o Wire.begin() esta la.

#define LASER_I2C_TIMEOUT_MS     500
#define LASER_TIMING_BUDGET_US   33000UL
#define LASER_MIN_VALID_MM       30
#define LASER_MAX_VALID_MM       2000
#define LASER_MAX_STEP_MM        50
#define LASER_FILTER_ALPHA       0.80

extern float laser_distance_mm;
extern boolean laser_distance_valid;
extern boolean laser_sensor_ok;

void LASER_setup();
void LASER_update();

#endif
