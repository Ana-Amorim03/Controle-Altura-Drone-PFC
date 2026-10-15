#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// Pinos ENA/ENB das duas pontes H L298N
#define MOTOR_ENA_P1   13
#define MOTOR_ENB_P1   12
#define MOTOR_ENA_P2   14
#define MOTOR_ENB_P2   27

// Pinos IN fixos: sentido de rotacao constante
#define MOTOR_PIN_IN_HIGH  26
#define MOTOR_PIN_IN_LOW   25

#define MOTOR_PWM_FREQ        1500
#define MOTOR_PWM_RESOLUTION  12
#define MOTOR_PWM_MAX_HW      4095

#define MOTOR_CH_M1  0
#define MOTOR_CH_M2  1
#define MOTOR_CH_M3  2
#define MOTOR_CH_M4  3

void MOTOR_setup();
void MOTOR_setPWM_individual(int pwm1, int pwm2, int pwm3, int pwm4);
void MOTOR_setPWM(int valor);

#endif
