#include <Arduino.h>
#include "MOTOR.h"

void MOTOR_setup() {
  pinMode(MOTOR_PIN_IN_HIGH, OUTPUT);
  pinMode(MOTOR_PIN_IN_LOW, OUTPUT);
  digitalWrite(MOTOR_PIN_IN_HIGH, HIGH);
  digitalWrite(MOTOR_PIN_IN_LOW, LOW);

  ledcSetup(MOTOR_CH_M1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcSetup(MOTOR_CH_M2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcSetup(MOTOR_CH_M3, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcSetup(MOTOR_CH_M4, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);

  ledcAttachPin(MOTOR_ENA_P1, MOTOR_CH_M2);
  ledcAttachPin(MOTOR_ENB_P1, MOTOR_CH_M3);
  ledcAttachPin(MOTOR_ENA_P2, MOTOR_CH_M4);
  ledcAttachPin(MOTOR_ENB_P2, MOTOR_CH_M1);

  MOTOR_setPWM(0);
}

void MOTOR_setPWM_individual(int pwm1, int pwm2, int pwm3, int pwm4) {
  ledcWrite(MOTOR_CH_M1, constrain(pwm1, 0, MOTOR_PWM_MAX_HW));
  ledcWrite(MOTOR_CH_M2, constrain(pwm2, 0, MOTOR_PWM_MAX_HW));
  ledcWrite(MOTOR_CH_M3, constrain(pwm3, 0, MOTOR_PWM_MAX_HW));
  ledcWrite(MOTOR_CH_M4, constrain(pwm4, 0, MOTOR_PWM_MAX_HW));
}

void MOTOR_setPWM(int valor) {
  MOTOR_setPWM_individual(valor, valor, valor, valor);
}
