// =====================================================================
// GY-85(ADXL345+ITG3200) + TB6612계열 드라이버(IN1/IN2/PWM/STBY) 버전
// ESP32 Super Mini, 모터 1개(단일 휠/1축) 기준, 전원 7.1V(2S급)
//
// 이번 버전 변경점:
// 1) 모터 회전방향 반전 (MOTOR_DIR_INVERT 스위치로 관리, 1=반전, 0=원래대로)
// 2) MIN_PWM을 10bit 해상도(0~500)에 맞게 60 -> 120으로 상향 (이전 8bit 기준값 그대로 쓰면
//    실제로는 절반 세기밖에 안 나가서 데드존 보정이 무력화됨)
//
// [배선 주의]
// - 모터드라이버 VM(모터전원) = 7.1V, VCC(로직전원) = ESP32와 같은 3.3V로 분리
// - GND는 전원/드라이버/ESP32 전부 공통으로 묶을 것
// =====================================================================

#include <Wire.h>
#include <Kalman.h>

// ---- 핀 정의 ----
#define SDA_PIN 2
#define SCL_PIN 3
#define IN1     4
#define IN2     5
#define PWM_PIN 6
#define STBY    7

#define PWM_FREQ 20000
#define PWM_RES  10
#define PWM_MAX  500

// ---- 방향 반전 스위치 : 1=반전(이번 테스트), 0=원래 방향으로 되돌릴 때 ----
#define MOTOR_DIR_INVERT 1

// ---- 모터 데드존(정지마찰) 보정값 ----
// 10bit(0~500) 기준으로 재조정. 여전히 안 움직이면 150~200까지 올려서 재실측 필요
#define MIN_PWM 120

// ---- GY-85 I2C 주소 ----
#define ADXL345_ADDR 0x53
#define ITG3200_ADDR 0x68

// ---- ADXL345 레지스터 ----
#define ADXL_REG_POWER_CTL 0x2D
#define ADXL_REG_DATA_FMT  0x31
#define ADXL_REG_DATAX0    0x32

// ---- ITG3200 레지스터 ----
#define ITG_REG_SMPLRT_DIV  0x15
#define ITG_REG_DLPF_FS     0x16
#define ITG_REG_PWR_MGM     0x3E
#define ITG_REG_GYRO_XOUT_H 0x1D

// ---- 시리얼 플로터 출력 설정 ----
#define PLOT_INTERVAL_MS 50
#define PLOT_SCALE_MAX  45.0f
#define PLOT_SCALE_MIN -45.0f
unsigned long lastPrintMs = 0;

unsigned long oldTime = 0, loopTime;
float dt;

// 튜닝 파라미터 (현재 테스트값: P-only)
float Kp = 80;
float Kd = 0;
int delayTime = 0;

float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float theta_X = 0.0;
float theta_Xdot = 0.0;
float offsetX = 0.0;

Kalman kalmanX;
float kalAngleX, kalAngleDotX;

int GetUP = 0;
float MtX;

// -------- I2C 저수준 헬퍼 --------
void i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *buf) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, count);
  for (int i = 0; i < count && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

// -------- ADXL345 초기화/읽기 --------
void adxl345_init() {
  i2cWriteReg(ADXL345_ADDR, ADXL_REG_DATA_FMT, 0x08);  // ±2g, full resolution
  i2cWriteReg(ADXL345_ADDR, ADXL_REG_POWER_CTL, 0x08); // 측정 모드 ON
}

void adxl345_read() {
  uint8_t buf[6];
  i2cReadBytes(ADXL345_ADDR, ADXL_REG_DATAX0, 6, buf);
  int16_t rawX = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t rawY = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t rawZ = (int16_t)((buf[5] << 8) | buf[4]);
  accX = rawX * 0.0039f; // ±2g, full-res: 1LSB = 3.9mg
  accY = rawY * 0.0039f;
  accZ = rawZ * 0.0039f;
}

// -------- ITG3200 초기화/읽기 --------
void itg3200_init() {
  i2cWriteReg(ITG3200_ADDR, ITG_REG_PWR_MGM, 0x00);    // 내부 클럭 사용
  i2cWriteReg(ITG3200_ADDR, ITG_REG_SMPLRT_DIV, 0x07); // 샘플레이트 분주
  i2cWriteReg(ITG3200_ADDR, ITG_REG_DLPF_FS, 0x1E);    // ±2000deg/s, LPF 20Hz
}

void itg3200_read() {
  uint8_t buf[6];
  i2cReadBytes(ITG3200_ADDR, ITG_REG_GYRO_XOUT_H, 6, buf);
  int16_t rawX = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t rawY = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t rawZ = (int16_t)((buf[4] << 8) | buf[5]);
  gyroX = rawX / 14.375f; // ITG3200 감도: 14.375 LSB/(deg/s)
  gyroY = rawY / 14.375f;
  gyroZ = rawZ / 14.375f;
}

// 가속도 기반 기울기(deg) 계산
void get_theta_acc() {
  adxl345_read();
  theta_X = atan2(accY, accZ) * 57.29578f + offsetX;
}

void get_gyro_data() {
  itg3200_read();
  theta_Xdot = gyroX;
}

// -------- 모터 구동 (TB6612 스타일 + 데드존 보정 + 방향 반전 스위치) --------
void motorWrite(float duty /* -1.0 ~ 1.0 */) {
  duty = constrain(duty, -1.0f, 1.0f);

#if MOTOR_DIR_INVERT
  duty = -duty; // 방향 반전: 부호만 뒤집어서 IN1/IN2 배선은 안 건드림
#endif

  int pwmVal = (int)(fabs(duty) * PWM_MAX);

  // 데드존 보정: duty가 0이 아닌데 정지마찰을 못 이길 만큼 작으면 최소값 보장
  if (duty != 0.0f && pwmVal < MIN_PWM) {
    pwmVal = MIN_PWM;
  }

  if (duty > 0.0f) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (duty < 0.0f) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  ledcWrite(PWM_PIN, pwmVal);
}

void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_PIN, 0);
}

// -------- 시리얼 튜닝 --------
void handleSerialTuning() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  int eq = line.indexOf('=');
  if (eq < 0) return;
  String key = line.substring(0, eq);
  float val = line.substring(eq + 1).toFloat();

  if (key == "Kp") Kp = val;
  else if (key == "Kd") Kd = val;
  else if (key == "offsetX") offsetX = val;
  else if (key == "delayTime") delayTime = (int)val;
}

// -------- 시리얼 플로터 출력 (50ms 간격, 상하한 기준선 포함) --------
void plotAngles() {
  unsigned long nowMs = millis();
  if (nowMs - lastPrintMs >= PLOT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    Serial.print(theta_X, 2);
    Serial.print(',');
    Serial.print(kalAngleX, 2);
    Serial.print(',');
    Serial.print(PLOT_SCALE_MAX, 0);
    Serial.print(',');
    Serial.println(PLOT_SCALE_MIN, 0);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // 드라이버 활성화

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);

  adxl345_init();
  itg3200_init();

  delay(50);
  get_theta_acc();
  kalmanX.setAngle(theta_X);

  motorStop();
}

void loop() {
  unsigned long nowTime = micros();
  loopTime = nowTime - oldTime;
  oldTime = nowTime;
  dt = (float)loopTime / 1000000.0;

  get_theta_acc();
  get_gyro_data();

  kalAngleX = kalmanX.getAngle(theta_X, theta_Xdot, dt);
  kalAngleDotX = kalmanX.getRate();

  if (fabs(kalAngleX) < 1 && GetUP == 0) {
    GetUP = 80;
  }

  if (GetUP == 80) {
    if (fabs(kalAngleX) > 15.0) {
      motorStop();
      GetUP = 0;
    } else {
      MtX = Kp * kalAngleX / 90.0 + Kd * kalAngleDotX / 500.0;
      motorWrite(MtX);
    }
  } else {
    motorStop();
  }

  handleSerialTuning();
  plotAngles();

  delay(delayTime);
}
