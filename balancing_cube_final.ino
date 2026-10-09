#include <Wire.h>
#include <Preferences.h>

// ============================================================
// ESP32-C3 SuperMini + GY-85 + TB6612FNG
// 1축 반작용휠 밸런싱 큐브 - EEPROM(NVS) 저장 + 반응속도 개선판
//
// 개선 내용
// 1) 기준각(balanceReference)을 Preferences(NVS)에 저장.
//    -> 손 떨림 섞인 캡처를 매번 반복하지 않음.
//    -> 시리얼로 'c' 입력했을 때만 재캡처 후 저장.
// 2) 평소 부팅 시: 큐브를 바닥/지그에 가만히 놓은 채로
//    자이로 캘리브레이션만 하고, 저장된 기준각을 바로 로드.
// 3) 반응 속도 개선:
//    - PWM_SLEW_PER_LOOP 증가 (더 빠른 토크 상승)
//    - GYRO_LPF_ALPHA 소폭 상향 (필터 지연 감소)
//    - 큰 오차에서 slew 제한을 우회하는 즉시반응(kick) 로직 추가
//
// 배선
// GY-85 SDA -> GPIO2, SCL -> GPIO3
// TB6612 AIN1+BIN1 -> GPIO4
// TB6612 AIN2+BIN2 -> GPIO5
// TB6612 PWMA+PWMB -> GPIO6
// TB6612 STBY      -> GPIO7
// ============================================================

// -------------------- 핀 정의 --------------------
#define SDA_PIN 2
#define SCL_PIN 3

#define IN1     4
#define IN2     5
#define PWM_PIN 6
#define STBY    7

#define PWM_FREQ 20000
#define PWM_RES  8

// -------------------- I2C 주소 --------------------
#define ADXL345_ADDR 0x53
#define ITG3200_ADDR 0x68

// -------------------- 제어 주기 / 필터 --------------------
const uint32_t LOOP_US = 5000;       // 200 Hz
const float COMP_ALPHA = 0.985f;     // 자이로 중심 상보필터
const float GYRO_LPF_ALPHA = 0.45f;  // 0.35 -> 0.45: 필터 지연 감소
const float GYRO_LSB_PER_DPS = 14.375f;

// -------------------- 튜닝 파라미터 --------------------
// 진동: KP 감소 또는 KD 증가.
// 반응이 약함: KP 증가, PWM_LIMIT 증가, 휠 관성 증가.
float KP = 35.0f;
float KD = 0.60f;

// TB6612/모터 발열을 보면서 220 -> 240 순으로 올릴 수 있음.
const int PWM_LIMIT = 220;

// 모터가 실제로 움직이기 시작하는 PWM. 방향 검증 끝났으므로 소량 적용.
const int MIN_EFFECTIVE_PWM = 20;

// 아주 작은 노이즈 명령은 제거.
const int PWM_DEADBAND = 2;

// 매 제어주기(5 ms) PWM 변화량. 20 -> 35로 상향: 더 빠른 토크 상승.
const int PWM_SLEW_PER_LOOP = 50;

// 오차가 큰 급박한 상황에서는 slew 제한을 우회하고 즉시 목표 PWM으로 점프.
// 이 각도(deg) 이상 벗어나면 즉시반응 모드로 전환.
const float KICK_ERROR_DEG = 6.0f;

// 검증 완료된 값. 큐브가 기울어진 방향으로 더 넘어지면 -1로 변경.
const int CONTROL_SIGN = -1;

// -------------------- 상태 전환 --------------------
const float ARM_ANGLE_DEG = 6.0f;      // 기준각 ±6도 안에서 ARM 대기
const float FALL_ANGLE_DEG = 28.0f;    // 이 이상 기울면 즉시 정지
const uint32_t ARM_HOLD_MS = 500;

// true: pitch + GyY, false: roll + GyX
const bool USE_PITCH_AXIS = true;

// -------------------- NVS(Preferences) --------------------
Preferences prefs;
const char *NVS_NAMESPACE = "balcube";
const char *NVS_KEY_REF = "refAngle";
const char *NVS_KEY_VALID = "refValid";

// -------------------- 전역 변수 --------------------
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

float gxFilt = 0.0f;
float gyFilt = 0.0f;
float gzFilt = 0.0f;

float roll = 0.0f;
float pitch = 0.0f;
float balanceReference = 0.0f;

int appliedPwm = 0;
bool armed = false;
uint32_t armCandidateMs = 0;

uint32_t lastLoopUs = 0;
uint32_t lastPrintMs = 0;

// -------------------- I2C 기본 함수 --------------------
void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

bool readBytes(uint8_t address, uint8_t startReg,
               uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(address);
  Wire.write(startReg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = Wire.requestFrom(address, length, (uint8_t)true);

  if (received != length) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

// -------------------- 센서 읽기 --------------------
bool readADXL345(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t data[6];

  if (!readBytes(ADXL345_ADDR, 0x32, data, 6)) {
    return false;
  }

  // ADXL345: little endian
  ax = (int16_t)((data[1] << 8) | data[0]);
  ay = (int16_t)((data[3] << 8) | data[2]);
  az = (int16_t)((data[5] << 8) | data[4]);

  return true;
}

bool readITG3200(int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t data[6];

  if (!readBytes(ITG3200_ADDR, 0x1D, data, 6)) {
    return false;
  }

  // ITG3200: big endian
  gx = (int16_t)((data[0] << 8) | data[1]);
  gy = (int16_t)((data[2] << 8) | data[3]);
  gz = (int16_t)((data[4] << 8) | data[5]);

  return true;
}

// -------------------- 센서 초기화 --------------------
void initSensors() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // GY-85 안정성 우선

  // ADXL345: full-resolution, ±2g, 200 Hz, measurement mode
  writeRegister(ADXL345_ADDR, 0x31, 0x08);
  writeRegister(ADXL345_ADDR, 0x2C, 0x0B);
  writeRegister(ADXL345_ADDR, 0x2D, 0x08);

  // ITG3200: sleep off, ±2000 dps, DLPF 42 Hz
  writeRegister(ITG3200_ADDR, 0x3E, 0x00);
  delay(100);
  writeRegister(ITG3200_ADDR, 0x16, 0x1E);
  writeRegister(ITG3200_ADDR, 0x15, 0x04);

  delay(200);
}

// -------------------- 자이로 영점 보정 --------------------
// 매번 필요함: 자이로 오프셋은 온도/시간에 따라 조금씩 변하므로
// 기준각과 달리 저장하지 않고 매 부팅마다 새로 측정.
void calibrateGyro() {
  const int SAMPLES = 1000;
  int32_t sumX = 0;
  int32_t sumY = 0;
  int32_t sumZ = 0;
  int valid = 0;

  Serial.println("Keep cube still on a flat surface: gyro warm-up 2 s");
  delay(2000);

  Serial.println("Gyro calibration: do not touch cube");

  for (int i = 0; i < SAMPLES; i++) {
    int16_t gx, gy, gz;

    if (readITG3200(gx, gy, gz)) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      valid++;
    }

    delay(2);
  }

  if (valid == 0) {
    Serial.println("FATAL: ITG3200 read failed");
    while (true) delay(1000);
  }

  gyroOffsetX = (float)sumX / valid;
  gyroOffsetY = (float)sumY / valid;
  gyroOffsetZ = (float)sumZ / valid;

  Serial.printf("Gyro offsets: %.1f, %.1f, %.1f\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
}

// -------------------- 가속도 각도 계산 --------------------
bool readAccelAngles(float &rollAcc, float &pitchAcc) {
  int16_t axRaw, ayRaw, azRaw;

  if (!readADXL345(axRaw, ayRaw, azRaw)) {
    return false;
  }

  const float ax = axRaw * 0.00390625f;
  const float ay = ayRaw * 0.00390625f;
  const float az = azRaw * 0.00390625f;

  rollAcc = atan2f(ay, az) * 180.0f / PI;
  pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  return isfinite(rollAcc) && isfinite(pitchAcc);
}

// -------------------- 기준 자세 새로 캡처 (수동 호출 전용) --------------------
void captureBalanceReference() {
  const int SAMPLES = 300;
  float sumRoll = 0.0f;
  float sumPitch = 0.0f;
  int valid = 0;

  Serial.println("=== CAPTURE MODE ===");
  Serial.println("Put cube at intended balance pose and hold firmly.");
  Serial.println("Capturing in 1.5 s...");
  delay(1500);

  for (int i = 0; i < SAMPLES; i++) {
    float r, p;

    if (readAccelAngles(r, p)) {
      sumRoll += r;
      sumPitch += p;
      valid++;
    }

    delay(5);
  }

  if (valid == 0) {
    Serial.println("FATAL: ADXL345 read failed during capture. Not saved.");
    return;
  }

  roll = sumRoll / valid;
  pitch = sumPitch / valid;
  balanceReference = USE_PITCH_AXIS ? pitch : roll;

  // NVS에 저장: 다음 부팅부터는 이 값을 그대로 사용
  prefs.putFloat(NVS_KEY_REF, balanceReference);
  prefs.putBool(NVS_KEY_VALID, true);

  Serial.printf("Captured & SAVED reference: %.2f deg (%s)\n",
                balanceReference,
                USE_PITCH_AXIS ? "PITCH" : "ROLL");
  Serial.println("=== CAPTURE DONE ===");
}

// -------------------- 저장된 기준 자세 로드 --------------------
bool loadBalanceReference() {
  bool valid = prefs.getBool(NVS_KEY_VALID, false);

  if (!valid) {
    return false;
  }

  balanceReference = prefs.getFloat(NVS_KEY_REF, 0.0f);
  return true;
}

// -------------------- 모터 제어 --------------------
void applyMotorPwm(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_PIN, pwm);

  } else if (pwm < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(PWM_PIN, -pwm);

  } else {
    // Coast: 휠을 자유롭게 감속시킴
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_PIN, 0);
  }
}

// 큰 오차에서는 slew 제한 없이 즉시 목표 PWM으로 점프해서 응답속도 확보
void setMotorTarget(int targetPwm, bool kick) {
  targetPwm = constrain(targetPwm, -PWM_LIMIT, PWM_LIMIT);

  if (kick) {
    appliedPwm = targetPwm;  // 즉시 반응

  } else if (targetPwm > appliedPwm + PWM_SLEW_PER_LOOP) {
    appliedPwm += PWM_SLEW_PER_LOOP;

  } else if (targetPwm < appliedPwm - PWM_SLEW_PER_LOOP) {
    appliedPwm -= PWM_SLEW_PER_LOOP;

  } else {
    appliedPwm = targetPwm;
  }

  applyMotorPwm(appliedPwm);
}

void stopMotorNow() {
  appliedPwm = 0;
  applyMotorPwm(0);
}

// 작은 PWM 명령이 모터 정지마찰을 못 이기는 문제 보상
int applyMinimumPwm(int pwm) {
  if (pwm == 0) return 0;

  if (abs(pwm) < MIN_EFFECTIVE_PWM) {
    return (pwm > 0) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
  }

  return pwm;
}

// -------------------- 시리얼 명령 처리 --------------------
// 'c' + Enter 입력 시에만 새로 기준각을 캡처하고 저장.
// 평소에는 절대 자동으로 재캡처하지 않음.
void handleSerialCommands() {
  if (!Serial.available()) return;

  char c = Serial.read();

  if (c == 'c' || c == 'C') {
    armed = false;
    armCandidateMs = 0;
    stopMotorNow();

    captureBalanceReference();

    // 재캡처 후 필터 상태 초기화
    roll = 0.0f;
    pitch = 0.0f;
    gxFilt = 0.0f;
    gyFilt = 0.0f;
    gzFilt = 0.0f;
  }
}

// -------------------- setup --------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=== One-axis Reaction Wheel Balance (EEPROM ver.) ===");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);  // ESP32-C3 새 LEDC API
  ledcWrite(PWM_PIN, 0);

  prefs.begin(NVS_NAMESPACE, false);  // read-write 모드로 NVS 오픈

  initSensors();
  calibrateGyro();  // 자이로 오프셋은 매번 새로 측정 (필수)

  // 저장된 기준각이 있으면 그대로 로드, 없으면 최초 1회 캡처 요구
  if (loadBalanceReference()) {
    Serial.printf("Loaded saved reference: %.2f deg (%s)\n",
                  balanceReference,
                  USE_PITCH_AXIS ? "PITCH" : "ROLL");

  } else {
    Serial.println("No saved reference found. Capturing for the first time.");
    captureBalanceReference();
  }

  Serial.println("Type 'c' + Enter anytime to re-capture & save reference pose.");

  gxFilt = 0.0f;
  gyFilt = 0.0f;
  gzFilt = 0.0f;
  roll = 0.0f;
  pitch = 0.0f;

  armed = false;
  armCandidateMs = 0;

  digitalWrite(STBY, HIGH);
  lastLoopUs = micros();

  Serial.println("Ready: keep near reference for 0.5 s.");
  Serial.println("angle,rate,error,target,pwm,armed,saturated");
}

// -------------------- loop --------------------
void loop() {
  handleSerialCommands();

  const uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastLoopUs) < LOOP_US) {
    return;
  }

  float dt = (nowUs - lastLoopUs) * 0.000001f;
  lastLoopUs = nowUs;

  dt = constrain(dt, 0.001f, 0.020f);

  int16_t axRaw, ayRaw, azRaw;
  int16_t gxRaw, gyRaw, gzRaw;

  const bool accOK = readADXL345(axRaw, ayRaw, azRaw);
  const bool gyroOK = readITG3200(gxRaw, gyRaw, gzRaw);

  if (!accOK || !gyroOK) {
    armed = false;
    armCandidateMs = 0;
    stopMotorNow();

    if (millis() - lastPrintMs >= 200) {
      lastPrintMs = millis();
      Serial.printf("I2C_FAIL acc=%d gyro=%d\n", accOK, gyroOK);
    }
    return;
  }

  const float ax = axRaw * 0.00390625f;
  const float ay = ayRaw * 0.00390625f;
  const float az = azRaw * 0.00390625f;

  const float rollAcc = atan2f(ay, az) * 180.0f / PI;
  const float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  if (!isfinite(rollAcc) || !isfinite(pitchAcc)) {
    armed = false;
    stopMotorNow();
    return;
  }

  const float gxDps = ((float)gxRaw - gyroOffsetX) / GYRO_LSB_PER_DPS;
  const float gyDps = ((float)gyRaw - gyroOffsetY) / GYRO_LSB_PER_DPS;
  const float gzDps = ((float)gzRaw - gyroOffsetZ) / GYRO_LSB_PER_DPS;

  gxFilt += GYRO_LPF_ALPHA * (gxDps - gxFilt);
  gyFilt += GYRO_LPF_ALPHA * (gyDps - gyFilt);
  gzFilt += GYRO_LPF_ALPHA * (gzDps - gzFilt);

  roll = COMP_ALPHA * (roll + gxFilt * dt)
       + (1.0f - COMP_ALPHA) * rollAcc;

  pitch = COMP_ALPHA * (pitch + gyFilt * dt)
        + (1.0f - COMP_ALPHA) * pitchAcc;

  const float angle = USE_PITCH_AXIS ? pitch : roll;
  const float rate = USE_PITCH_AXIS ? gyFilt : gxFilt;
  const float error = balanceReference - angle;

  const float absError = fabsf(error);
  const uint32_t nowMs = millis();

  if (!armed) {
    stopMotorNow();

    if (absError < ARM_ANGLE_DEG) {
      if (armCandidateMs == 0) {
        armCandidateMs = nowMs;
      }

      if (nowMs - armCandidateMs >= ARM_HOLD_MS) {
        armed = true;
        Serial.println("[STATE] ARMED");
      }
    } else {
      armCandidateMs = 0;
    }

  } else if (absError > FALL_ANGLE_DEG) {
    armed = false;
    armCandidateMs = 0;
    stopMotorNow();
    Serial.println("[STATE] FALL / DISARM");
  }

  int targetPwm = 0;
  bool saturated = false;

  if (armed) {
    float control = CONTROL_SIGN * (KP * error - KD * rate);

    if (fabsf(control) >= PWM_LIMIT) {
      saturated = true;
    }

    targetPwm = (int)lroundf(control);
    targetPwm = constrain(targetPwm, -PWM_LIMIT, PWM_LIMIT);

    if (abs(targetPwm) <= PWM_DEADBAND) {
      targetPwm = 0;
    } else {
      targetPwm = applyMinimumPwm(targetPwm);
    }

    // 오차가 크면 slew 제한을 우회해 즉시 반응 (반응속도 개선 핵심)
    bool kick = (absError >= KICK_ERROR_DEG);

    setMotorTarget(targetPwm, kick);
  }

  if (nowMs - lastPrintMs >= 50) {
    lastPrintMs = nowMs;

    Serial.print(angle, 2);
    Serial.print(",");
    Serial.print(rate, 2);
    Serial.print(",");
    Serial.print(error, 2);
    Serial.print(",");
    Serial.print(targetPwm);
    Serial.print(",");
    Serial.print(appliedPwm);
    Serial.print(",");
    Serial.print(armed ? 1 : 0);
    Serial.print(",");
    Serial.println(saturated ? 1 : 0);
  }
}
