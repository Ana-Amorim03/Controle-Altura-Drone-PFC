// Baseado no tutorial de Joop Brokking para o MPU-6050
// (http://www.brokking.net/imu.html).

#include <Wire.h>
#include "IMU.h"

static const int MPU_ADDR = 0x68;
static const float GYRO_LSB_PER_DEGS = 65.5;
static const float RATE_FILTER_ALPHA = 0.85;

// 1 / (250 Hz * 65.5) e o mesmo valor convertido para radianos
static const double GYRO_DEG_POR_LSB_AMOSTRA = 0.0000611;
static const double GYRO_RAD_POR_LSB_AMOSTRA = 0.000001066;

static int gyro_x, gyro_y, gyro_z;
static long gyro_x_cal, gyro_y_cal, gyro_z_cal;
static long acc_x, acc_y, acc_z, acc_total_vector;
static int temperature;

static float angle_pitch, angle_roll;
static float angle_pitch_acc, angle_roll_acc;
static boolean set_gyro_angles;

float angle_pitch_output, angle_roll_output;
float angle_yaw;
float rate_p, rate_q, rate_r;

static void write_register(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  Wire.endTransmission();
}

static void setup_mpu_6050_registers() {
  write_register(0x6B, 0x00);   // sai do modo sleep
  write_register(0x1C, 0x10);   // acelerometro +-8 g
  write_register(0x1B, 0x08);   // giroscopio +-500 graus/s
}

static void read_mpu_6050_data() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 14);
  while (Wire.available() < 14);
  acc_x = (int16_t)(Wire.read() << 8 | Wire.read());
  acc_y = (int16_t)(Wire.read() << 8 | Wire.read());
  acc_z = (int16_t)(Wire.read() << 8 | Wire.read());
  temperature = (int16_t)(Wire.read() << 8 | Wire.read());
  gyro_x = (int16_t)(Wire.read() << 8 | Wire.read());
  gyro_y = (int16_t)(Wire.read() << 8 | Wire.read());
  gyro_z = (int16_t)(Wire.read() << 8 | Wire.read());
}

void IMU_setup() {
  Wire.begin();
  setup_mpu_6050_registers();

  for (int i = 0; i < 2000; i++) {
    if (i % 125 == 0) Serial.print(".");
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delay(3);
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
}

void IMU_update() {
  read_mpu_6050_data();

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  angle_pitch += gyro_x * GYRO_DEG_POR_LSB_AMOSTRA;
  angle_roll += gyro_y * GYRO_DEG_POR_LSB_AMOSTRA;

  // acoplamento entre pitch e roll quando a IMU gira em yaw
  angle_pitch += angle_roll * sin(gyro_z * GYRO_RAD_POR_LSB_AMOSTRA);
  angle_roll -= angle_pitch * sin(gyro_z * GYRO_RAD_POR_LSB_AMOSTRA);

  float rate_p_raw = (gyro_x / GYRO_LSB_PER_DEGS) * DEG_TO_RAD;
  float rate_q_raw = (gyro_y / GYRO_LSB_PER_DEGS) * DEG_TO_RAD;
  float rate_r_raw = (gyro_z / GYRO_LSB_PER_DEGS) * DEG_TO_RAD;

  rate_p = rate_p * RATE_FILTER_ALPHA + rate_p_raw * (1.0 - RATE_FILTER_ALPHA);
  rate_q = rate_q * RATE_FILTER_ALPHA + rate_q_raw * (1.0 - RATE_FILTER_ALPHA);
  rate_r = rate_r * RATE_FILTER_ALPHA + rate_r_raw * (1.0 - RATE_FILTER_ALPHA);

  angle_yaw += gyro_z * GYRO_DEG_POR_LSB_AMOSTRA;

  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
  float acc_y_ratio = constrain((float)acc_y / acc_total_vector, -1.0, 1.0);
  float acc_x_ratio = constrain((float)acc_x / acc_total_vector, -1.0, 1.0);
  angle_pitch_acc = asin(acc_y_ratio) * 57.296;
  angle_roll_acc = asin(acc_x_ratio) * -57.296;

  if (set_gyro_angles && acc_total_vector != 0) {
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  } else if (!set_gyro_angles && acc_total_vector != 0) {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }

  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;

  if (isnan(angle_pitch) || isnan(angle_roll) || isnan(angle_pitch_output) || isnan(angle_roll_output)) {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    angle_pitch_output = angle_pitch_acc;
    angle_roll_output = angle_roll_acc;
  }
}
