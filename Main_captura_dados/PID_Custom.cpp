// Baseado na Arduino PID Library 1.3.0, de Brett Beauregard (MIT).
// Alteracoes: tempo em micros(), filtro passa-baixa no termo derivativo
// e anti-windup por clamping (integracao condicional). 
// Como citado na monografia e no README, essa biblioteca foi adaptada por André Ferreira

#include <Arduino.h>
#include "PID_Custom.h"

PID::PID(double* input, double* output, double* setpoint,
         double Kp, double Ki, double Kd)
{
  myInput = input;
  myOutput = output;
  mySetpoint = setpoint;
  inAuto = false;
  filterN = 100.0;
  filteredDInput = 0.0;
  outputSum = 0.0;
  outMin = 0;
  outMax = 255;
  sampleTime = 4000;

  double ts = sampleTime / 1000000.0;
  kp = Kp;
  ki = Ki * ts;
  kd = Kd / ts;

  lastTime = micros() - sampleTime;
}

bool PID::Compute()
{
  if (!inAuto) return false;

  unsigned long now = micros();
  if (now - lastTime < sampleTime) return false;

  double ts = sampleTime / 1000000.0;
  double input = *myInput;
  double error = *mySetpoint - input;

  double dTermRaw = filterN * (input - filteredDInput);
  filteredDInput += dTermRaw * ts;

  double pTerm = kp * error;
  double dTerm = -kd * dTermRaw;
  double candidato = pTerm + outputSum + dTerm;

  bool saturadoAlto  = (candidato >= outMax) && (error > 0);
  bool saturadoBaixo = (candidato <= outMin) && (error < 0);

  if (!saturadoAlto && !saturadoBaixo) {
    outputSum += ki * error;
    outputSum = constrain(outputSum, outMin, outMax);
  }

  *myOutput = constrain(pTerm + outputSum + dTerm, outMin, outMax);

  lastTime = now;
  return true;
}

void PID::SetSampleTime(unsigned long novo_periodo_us)
{
  if (novo_periodo_us == 0) return;
  double razao = (double)novo_periodo_us / (double)sampleTime;
  ki *= razao;
  kd /= razao;
  sampleTime = novo_periodo_us;
}

void PID::SetDerivativeFilter(double n)
{
  if (n > 0) filterN = n;
}

void PID::SetOutputLimits(double min, double max)
{
  if (min >= max) return;
  outMin = min;
  outMax = max;

  if (inAuto) {
    *myOutput = constrain(*myOutput, outMin, outMax);
    outputSum = constrain(outputSum, outMin, outMax);
  }
}

void PID::SetMode(int modo)
{
  bool novoAuto = (modo == AUTOMATIC);
  if (novoAuto && !inAuto) Initialize();
  inAuto = novoAuto;
}

int PID::GetMode()
{
  return inAuto ? AUTOMATIC : MANUAL;
}

void PID::Initialize()
{
  outputSum = constrain(*myOutput, outMin, outMax);
  filteredDInput = *myInput;
}
