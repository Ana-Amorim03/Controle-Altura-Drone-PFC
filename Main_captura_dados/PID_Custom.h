#ifndef PID_Custom_h
#define PID_Custom_h

#define AUTOMATIC 1
#define MANUAL    0

// Controlador PID baseado na Arduino PID Library 1.3.0 (Brett Beauregard, MIT).
// Tempo em microssegundos, filtro no termo derivativo e anti-windup por clamping.
class PID {
public:
  PID(double* input, double* output, double* setpoint,
      double kp, double ki, double kd);

  bool Compute();
  void SetMode(int modo);
  int GetMode();
  void SetOutputLimits(double min, double max);
  void SetSampleTime(unsigned long novo_periodo_us);
  void SetDerivativeFilter(double n);

private:
  void Initialize();

  double kp, ki, kd;
  double *myInput, *myOutput, *mySetpoint;
  unsigned long lastTime, sampleTime;
  double outputSum, filteredDInput, filterN;
  double outMin, outMax;
  bool inAuto;
};

#endif
