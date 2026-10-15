#include <Wire.h>
#include <VL53L0X.h>
#include "LASER.h"

static VL53L0X laser_sensor;
static float laser_distance_raw_mm = 0;

float laser_distance_mm = 0;
boolean laser_distance_valid = false;
boolean laser_sensor_ok = false;

void LASER_setup() {
  laser_sensor.setTimeout(LASER_I2C_TIMEOUT_MS);

  if (!laser_sensor.init()) {
    laser_sensor_ok = false;
    Serial.println("Falha ao iniciar o VL53L0X. Verifique a fiacao I2C.");
    return;
  }

  laser_sensor.setMeasurementTimingBudget(LASER_TIMING_BUDGET_US);
  laser_sensor.startContinuous();
  laser_sensor_ok = true;
}

// Le a medida apenas se ja estiver pronta, para nao bloquear o loop de 250 Hz.
void LASER_update() {
  if (!laser_sensor_ok) return;

  if ((laser_sensor.readReg(laser_sensor.RESULT_INTERRUPT_STATUS) & 0x07) == 0) return;

  uint16_t range_raw = laser_sensor.readReg16Bit(laser_sensor.RESULT_RANGE_STATUS + 10);
  laser_sensor.writeReg(laser_sensor.SYSTEM_INTERRUPT_CLEAR, 0x01);

  if (laser_sensor.timeoutOccurred()) {
    laser_distance_valid = false;
    return;
  }

  if (range_raw < LASER_MIN_VALID_MM || range_raw > LASER_MAX_VALID_MM) {
    laser_distance_valid = false;
    return;
  }

  if (laser_distance_valid && fabs((float)range_raw - laser_distance_raw_mm) > LASER_MAX_STEP_MM) {
    return;
  }

  laser_distance_raw_mm = (float)range_raw;

  if (!laser_distance_valid) {
    laser_distance_mm = laser_distance_raw_mm;
  } else {
    laser_distance_mm = laser_distance_mm * LASER_FILTER_ALPHA + laser_distance_raw_mm * (1.0 - LASER_FILTER_ALPHA);
  }

  laser_distance_valid = true;
}
