// Controle de altura com telemetria em CSV pela serial.
// Usar com capturar_serial.py e plotar_dados.py. O quadricoptero deve estar
// preso as hastes-guia; a IMU so e usada para inicializar o barramento I2C, nao esta sendo captado nenhum dado.

#include <Wire.h>
#include <esp_task_wdt.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "IMU.h"
#include "LASER.h"
#include "MOTOR.h"
#include "PID_Custom.h"

// Watchdog e pinos do barramento I2C
const int WDT_TIMEOUT_S = 3;
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// Periodo do laco de controle (250 Hz)
const unsigned long PERIODO_US = 4000;
const double TS_S = 0.004;

// Parametros da planta
const double MASSA_KG = 0.0173;
const double G = 9.81;
const double K_SUSTENTACAO = 7.72e-9;

// Constantes de cada motor: omega = c * PWM + c0
const double C_PWM[4]  = {0.31395, 0.29816, 0.28865, 0.33086};
const double C0_PWM[4] = {1373.17, 1532.71, 1452.61, 1409.18};

// Protecoes e filtros do sinal de controle
const int PWM_LIMITE_SEGURANCA = 4000;
const double PWM_SLEW_MAX_POR_CICLO = 40.0;
const double SETPOINT_RAMPA_MAX_M_S = 0.02;
const double DEADBAND_M = 0.002;
const double SALTO_MAX_ALTURA_M = 0.03;

// Ganhos do controlador PI de altura
double Kp_z = 0.23, Ki_z = 0.16, Kd_z = 0.0;

// Estado do controle de altura (metros)
double input_altura_m;
double output_pid_z;
double setpoint_altura_m = 0.02;
double setpoint_suave_m = 0.0;
double offset_base_mm = 0;
double pwm_suave[4] = {0, 0, 0, 0};

PID pid_altura(&input_altura_m, &output_pid_z, &setpoint_suave_m, Kp_z, Ki_z, Kd_z);

unsigned long loop_timer;
int contador_telemetria = 0;

// Em caso de travamento, destrava o barramento I2C enviando pulsos manuais de clock e reinicia o Wire.
void recuperar_barramento_i2c() {
  Wire.end();
  pinMode(I2C_SCL_PIN, OUTPUT);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);

  for (int i = 0; i < 16; i++) {
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    if (digitalRead(I2C_SDA_PIN) == HIGH) break;
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setTimeOut(50);
  Serial.println("[I2C] Barramento recuperado.");
}

// O zero da altura e a media das leituras com o drone apoiado na base.
void calibrar_offset_altura() {
  const int N_AMOSTRAS = 30;
  long soma_mm = 0;
  int validas = 0;
  unsigned long inicio = millis();

  while (validas < N_AMOSTRAS && millis() - inicio < 3000) {
    LASER_update();
    if (laser_distance_valid) {
      soma_mm += laser_distance_mm;
      validas++;
    }
    delay(20);
  }

  if (validas > 0) {
    offset_base_mm = (double)soma_mm / validas;
    Serial.print("Offset da altura: ");
    Serial.print(offset_base_mm, 1);
    Serial.println(" mm");
  } else {
    offset_base_mm = 0;
    Serial.println("Sem leituras validas, offset = 0.");
  }
}

// Inicializa os sensores e os motores, calibra a altura e liga o watchdog.
// O controle so comeca quando o comando 'a' e recebido.
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Comandos: numero (cm) = setpoint, 'a' = ativar controle, 's' = parar motores.");

  IMU_setup();
  Wire.setTimeOut(50);
  LASER_setup();
  if (!laser_sensor_ok) Serial.println("ALERTA: laser nao inicializado.");

  MOTOR_setup();

  Serial.println("Calibrando a altura (drone parado na base)...");
  calibrar_offset_altura();

  pid_altura.SetSampleTime(PERIODO_US);
  pid_altura.SetOutputLimits(-MASSA_KG * G, MASSA_KG * G);
  pid_altura.SetMode(MANUAL);

  // O watchdog so e ligado depois das calibracoes, que demoram mais que o timeout.
  // O loop principal usa espera ocupada e deixa as tarefas IDLE sem executar,
  // por isso elas sao retiradas do monitoramento.
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));
  esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(1));

  Serial.println("t_ms,altura_cm,setpoint_cm,u1_N,pwm_m1,pwm_m2,pwm_m3,pwm_m4");
  Serial.print("#PARAMS,");
  Serial.print(Kp_z, 4); Serial.print(",");
  Serial.print(Ki_z, 4); Serial.print(",");
  Serial.println(Kd_z, 4);

  loop_timer = micros();
}

// Trata os comandos recebidos pela serial: numero em cm (setpoint),
// 'a' (ativa o controle) e 's' (desliga os motores).
void processar_comando_serial() {
  if (Serial.available() <= 0) return;

  char c = Serial.peek();

  if (c == 's' || c == 'S') {
    Serial.read();
    MOTOR_setPWM(0);
    pid_altura.SetMode(MANUAL);
    Serial.println(">> Motores desligados.");
  } else if (c == 'a' || c == 'A') {
    Serial.read();
    pid_altura.SetMode(AUTOMATIC);
    Serial.println(">> Controle ativado.");
  } else if (isDigit(c) || c == '-') {
    double valor_cm = Serial.parseFloat();
    while (Serial.available() > 0) Serial.read();
    setpoint_altura_m = valor_cm / 100.0;
    Serial.print(">> Setpoint: ");
    Serial.print(valor_cm, 1);
    Serial.println(" cm");
  } else {
    Serial.read();
  }
}

// Reinicia o barramento se o laser ficar 500 ms sem leitura valida.
void verificar_barramento_i2c() {
  static unsigned long ultima_leitura_ms = 0;
  static bool ja_recuperou = false;

  if (laser_distance_valid) {
    ultima_leitura_ms = millis();
    ja_recuperou = false;
  } else if (millis() - ultima_leitura_ms > 500 && !ja_recuperou) {
    Serial.println("[I2C] Sem leituras ha 500 ms, recuperando o barramento.");
    recuperar_barramento_i2c();
    LASER_setup();
    ja_recuperou = true;
    ultima_leitura_ms = millis();
  }
}

// Saltos maiores que 3 cm entre amostras (250 Hz) sao tratados como glitch.
// Converte a leitura do laser em altura (m) e descarta leituras com salto brusco.
void atualizar_altura() {
  if (!laser_distance_valid) return;

  double nova_m = (laser_distance_mm - offset_base_mm) / 1000.0;
  if (fabs(nova_m - input_altura_m) < SALTO_MAX_ALTURA_M || input_altura_m == 0) {
    input_altura_m = max(nova_m, 0.0);
  }
}

// Calcula o PWM de cada motor: compensacao de gravidade + PI, dividido
// igualmente entre os motores, com limite de taxa de variacao.
double controlar_altura(int pwm[4]) {
  // rampa de referencia: o setpoint usado pelo PI se aproxima do desejado aos poucos
  double passo_max_m = SETPOINT_RAMPA_MAX_M_S * TS_S;
  double diferenca = setpoint_altura_m - setpoint_suave_m;
  if (diferenca > passo_max_m) setpoint_suave_m += passo_max_m;
  else if (diferenca < -passo_max_m) setpoint_suave_m -= passo_max_m;
  else setpoint_suave_m = setpoint_altura_m;

  // dentro da banda morta o erro e zerado
  if (fabs(setpoint_altura_m - input_altura_m) < DEADBAND_M) {
    input_altura_m = setpoint_altura_m;
  }
  pid_altura.Compute();

  // empuxo total = compensacao de gravidade + correcao do PI
  double u1 = constrain(MASSA_KG * G + output_pid_z, 0.0, 2 * MASSA_KG * G);
  // velocidade angular igual nos 4 motores; o PWM difere pelas constantes de cada um
  double omega = sqrt(u1 / (4.0 * K_SUSTENTACAO));

  for (int i = 0; i < 4; i++) {
    // limita a variacao do PWM por ciclo para evitar picos de corrente
    double alvo = constrain((omega - C0_PWM[i]) / C_PWM[i], 0.0, (double)PWM_LIMITE_SEGURANCA);
    double diferenca_pwm = alvo - pwm_suave[i];
    if (diferenca_pwm > PWM_SLEW_MAX_POR_CICLO) pwm_suave[i] += PWM_SLEW_MAX_POR_CICLO;
    else if (diferenca_pwm < -PWM_SLEW_MAX_POR_CICLO) pwm_suave[i] -= PWM_SLEW_MAX_POR_CICLO;
    else pwm_suave[i] = alvo;
    pwm[i] = (int)round(pwm_suave[i]);
  }
  return u1;
}

// Envia uma linha CSV: tempo, altura, setpoint, empuxo total e PWM dos motores.
void enviar_telemetria(double u1, const int pwm[4]) {
  Serial.print(millis());                      Serial.print(",");
  Serial.print(input_altura_m * 100.0, 3);     Serial.print(",");
  Serial.print(setpoint_altura_m * 100.0, 3);  Serial.print(",");
  Serial.print(u1, 4);                         Serial.print(",");
  Serial.print(pwm[0]); Serial.print(",");
  Serial.print(pwm[1]); Serial.print(",");
  Serial.print(pwm[2]); Serial.print(",");
  Serial.println(pwm[3]);
}

// Ciclo de 4 ms: comandos, leitura dos sensores, controle, motores e telemetria.
void loop() {
  esp_task_wdt_reset();

  processar_comando_serial();

  IMU_update();
  LASER_update();
  verificar_barramento_i2c();
  atualizar_altura();

  // com o controle desativado os motores ficam parados
  double u1 = 0;
  int pwm[4] = {0, 0, 0, 0};

  if (pid_altura.GetMode() == AUTOMATIC) {
    u1 = controlar_altura(pwm);
  } else {
    for (int i = 0; i < 4; i++) pwm_suave[i] = 0;
  }

  MOTOR_setPWM_individual(pwm[0], pwm[1], pwm[2], pwm[3]);

  // telemetria a cada 10 ciclos (25 Hz)
  if (++contador_telemetria == 10) {
    contador_telemetria = 0;
    enviar_telemetria(u1, pwm);
  }

  while (micros() - loop_timer < PERIODO_US);
  loop_timer = micros();
}
