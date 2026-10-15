# Controle da Altura de um Drone — PFC

Códigos do Projeto de Fim de Curso **"Desenvolvimento de uma Planta Didática para Controle de Altura de um Drone"**, de Ana Paula Amorim Almeida (Engenharia de Controle e Automação, UFMG).

A planta é um quadricóptero preso a hastes-guia, de modo que apenas o movimento vertical é livre. A altura é controlada por um PI, executado a 250 Hz em um ESP32.

<p align="center">
  <img src="https://github.com/user-attachments/assets/6b196af2-96ec-4509-a170-4629493a1403" alt="Planta, vista lateral" width="320">
  <img src="https://github.com/user-attachments/assets/97ae9be8-c970-45fb-9eb1-03cd7a5c9ae9" alt="Drone, vista superior" width="360">
</p>


## Hardware

- ESP32
- Sensor de distância laser VL53L0X (altura)
- IMU MPU-6050 (ângulos; neste teste é usada apenas para manter o barramento I2C ativo)
- Duas pontes H L298N e quatro motores
- Fonte de 9 V

## Estrutura

| Arquivo | Função |
|---|---|
| `Main_captura_dados.ino` | Programa principal: controle de altura e telemetria em CSV |
| `IMU.cpp` / `IMU.h` | Leitura e filtro complementar da MPU-6050 |
| `LASER.cpp` / `LASER.h` | Leitura e filtro do VL53L0X |
| `MOTOR.cpp` / `MOTOR.h` | PWM dos quatro motores |
| `PID_Custom.cpp` / `PID_Custom.h` | PID discreto com anti-windup e filtro derivativo |
| `capturar_serial.py` | Salva a telemetria serial em `.csv` e permite enviar comandos |
| `plotar_dados.py` | Gera os gráficos a partir do `.csv` |

## Como usar

1. Fixar o quadricóptero às hastes, apoiado na base, e ligar a fonte de 9 V.
2. Carregar `Main_captura_dados.ino` pela IDE do Arduino (placa ESP32, biblioteca VL53L0X da Pololu).
3. Aguardar a calibração (cerca de 6 s) sem mexer no drone.
4. Enviar comandos pela serial (115200 baud):

| Comando | Ação |
|---|---|
| `a` | Ativa o controle de altura |
| número (cm) | Define a altura de referência, por exemplo `5` |
| `s` | Parada de emergência (desliga os motores) |

A referência inicial é de 2 cm e as mudanças são aplicadas com rampa de 2 cm/s.

## Captura de dados

```bash
pip install pyserial pandas matplotlib
# editar a variável PORTA em capturar_serial.py
python capturar_serial.py
python plotar_dados.py dados_AAAAMMDD_HHMMSS.csv
```

Feche o monitor serial da IDE antes de rodar a captura. Pare com `Ctrl+C`.

## Parâmetros principais

Definidos no início de `Main_captura_dados.ino`: ganhos `Kp_z` e `Ki_z`, limite de PWM (`PWM_LIMITE_SEGURANCA`), variação máxima por ciclo (`PWM_SLEW_MAX_POR_CICLO`), rampa da referência (`SETPOINT_RAMPA_MAX_M_S`) e banda morta (`DEADBAND_M`).

## Créditos

- PID baseado na [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library), de Brett Beauregard, adaptada por André Ferreira (anti-windup).
- IMU baseada no tutorial de Joop Brokking: <http://www.brokking.net/imu.html>.
- Leitura do laser com a biblioteca VL53L0X da Pololu.
