#include <Wire.h>
#include <Preferences.h>

// ============================================================
// smallcube_v5.ino
// ESP32-C3 SuperMini + GY-85 + TB6612FNG
// 1축 반작용휠 밸런싱 큐브
//
// v5 변경점
// - I2C를 GPIO2(SDA), GPIO3(SCL)로 사용
// - GPIO8/9는 이 보드에서 LED/BOOT 회로와 연결되어 있으므로 사용하지 않음
// - 나머지 제어 구조는 현재 잘 동작하는 v4 설정 유지
// ============================================================

// -------------------- 핀 정의 --------------------
#define SDA_PIN 2  // GY-85 SDA
#define SCL_PIN 3  // GY-85 SCL

#define IN1     4
#define IN2     5
#define PWM_PIN 6
#define STBY    7

#define PWM_FREQ 20000
#define PWM_RES  8

// -------------------- I2C 주소 --------------------
#define ADXL345_ADDR 0x53
#define ITG3200_ADDR 0x68

// -------------------- 제어 주기 / IMU 필터 --------------------
const uint32_t LOOP_US = 5000;       // 200 Hz 제어 주기
const float COMP_ALPHA = 0.97f;     // 상보필터 자이로 비중
const float GYRO_LPF_ALPHA = 0.75f;  // 자이로 저역통과필터
const float GYRO_LSB_PER_DPS = 14.375f;

// -------------------- 제어 게인 --------------------
float KP = 120.0f;   // 자세 오차 복원력
float KD = 8.0f;   // 프레임 각속도 감쇠

// 엔코더 없는 휠 상태 대용 피드백
float KW = 0.03f;    // 모터 명령 누적 상태 억제
float IDRS = 0.02f;  // 장시간 한 방향 구동 시 기준각 미세 보정

const float MOTOR_MEMORY_ALPHA = 0.05f; // PWM 기반 휠 상태 추정 필터
const float WHEEL_TRIM_LIMIT_DEG = 2.0f; // 기준각 보정 최대 범위

// -------------------- PWM / 모터 설정 --------------------
const int PWM_LIMIT = 240;         // 최대 PWM 제한
const int PWM_DEADBAND = 2;        // 미세 명령 무시 구간
const int MIN_EFFECTIVE_PWM = 18;  // 코어리스 모터 기동 최소 PWM
const int PWM_SLEW_PER_LOOP = 130; // 현재는 즉시 출력
const float KICK_ERROR_DEG = 0.0f; // 현재는 항상 즉시 출력

// -------------------- 제어 / 안전 설정 --------------------
const int CONTROL_SIGN = 1; // 현재 실험에서 복원 방향이 맞는 부호
const float ARM_ANGLE_DEG = 4.0f;
const float FALL_ANGLE_DEG = 28.0f;
const uint32_t ARM_HOLD_MS = 500;

// true: pitch + GyY, false: roll + GyX
const bool USE_PITCH_AXIS = true;

// -------------------- NVS 기준각 저장 --------------------
Preferences prefs;
const char *NVS_NAMESPACE = "balcube";
const char *NVS_KEY_REF = "refAngle";
const char *NVS_KEY_VALID = "refValid";

// -------------------- 센서 상태 --------------------
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

float gxFilt = 0.0f;
float gyFilt = 0.0f;
float gzFilt = 0.0f;

float roll = 0.0f;
float pitch = 0.0f;
float balanceReference = 0.0f;

// -------------------- 제어 상태 --------------------
int appliedPwm = 0;
float motorMemory = 0.0f;    // 엔코더 없는 Mpre 대용값
float wheelAngleTrim = 0.0f; // IDRS 누적 기준각 보정

bool armed = false;
uint32_t armCandidateMs = 0;
uint32_t lastLoopUs = 0;
uint32_t lastPrintMs = 0;

// ============================================================
// I2C 기본 함수
// ============================================================
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

  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom(address, length, (uint8_t)true);
  if (received != length) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (uint8_t i = 0; i < length; i++) buffer[i] = Wire.read();
  return true;
}

// I2C bus가 SDA LOW 상태로 멈췄을 때 최대 9회 SCL pulse로 복구
void recoverI2CBus() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);

  for (uint8_t i = 0; i < 9 && digitalRead(SDA_PIN) == LOW; i++) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
  }

  // STOP 조건 생성
  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(5);

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
}

// ============================================================
// GY-85 센서 함수
// ============================================================
bool readADXL345(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t data[6];
  if (!readBytes(ADXL345_ADDR, 0x32, data, 6)) return false;

  // ADXL345는 little-endian
  ax = (int16_t)((data[1] << 8) | data[0]);
  ay = (int16_t)((data[3] << 8) | data[2]);
  az = (int16_t)((data[5] << 8) | data[4]);
  return true;
}

bool readITG3200(int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t data[6];
  if (!readBytes(ITG3200_ADDR, 0x1D, data, 6)) return false;

  // ITG3200은 big-endian
  gx = (int16_t)((data[0] << 8) | data[1]);
  gy = (int16_t)((data[2] << 8) | data[3]);
  gz = (int16_t)((data[4] << 8) | data[5]);
  return true;
}

bool readAccelAngles(float &rollAcc, float &pitchAcc) {
  int16_t axRaw, ayRaw, azRaw;
  if (!readADXL345(axRaw, ayRaw, azRaw)) return false;

  const float ax = axRaw * 0.00390625f;
  const float ay = ayRaw * 0.00390625f;
  const float az = azRaw * 0.00390625f;

  rollAcc = atan2f(ay, az) * 180.0f / PI;
  pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  return isfinite(rollAcc) && isfinite(pitchAcc);
}

void initSensors() {
  recoverI2CBus();
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // GY-85 안정성 우선

  // ADXL345: full resolution, ±2g, 200 Hz, measurement mode
  writeRegister(ADXL345_ADDR, 0x31, 0x08);
  writeRegister(ADXL345_ADDR, 0x2C, 0x0B);
  writeRegister(ADXL345_ADDR, 0x2D, 0x08);

  // ITG3200: ±2000 dps, DLPF 42 Hz, sample divider 4
  writeRegister(ITG3200_ADDR, 0x3E, 0x00);
  delay(100);
  writeRegister(ITG3200_ADDR, 0x16, 0x1E);
  writeRegister(ITG3200_ADDR, 0x15, 0x04);
  delay(200);
}

void calibrateGyro() {
  const int SAMPLES = 1000;
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  int valid = 0;

  Serial.println("Keep cube still on flat surface: gyro warm-up 2 s");
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

// ============================================================
// 기준각 NVS 저장/로드
// ============================================================
void captureBalanceReference() {
  const int SAMPLES = 500;
  float sumRoll = 0.0f, sumPitch = 0.0f;
  int valid = 0;

  Serial.println("=== CAPTURE MODE ===");
  Serial.println("Hold the cube at its real edge equilibrium pose.");
  Serial.println("Capture starts in 2 seconds...");
  delay(2000);

  for (int i = 0; i < SAMPLES; i++) {
    float r, p;
    if (readAccelAngles(r, p)) {
      sumRoll += r;
      sumPitch += p;
      valid++;
    }
    delay(4);
  }

  if (valid == 0) {
    Serial.println("FATAL: ADXL345 read failed. Reference was NOT saved.");
    return;
  }

  roll = sumRoll / valid;
  pitch = sumPitch / valid;
  balanceReference = USE_PITCH_AXIS ? pitch : roll;

  prefs.putFloat(NVS_KEY_REF, balanceReference);
  prefs.putBool(NVS_KEY_VALID, true);

  Serial.printf("Captured and SAVED reference: %.3f deg (%s)\n",
                balanceReference, USE_PITCH_AXIS ? "PITCH" : "ROLL");
}

bool loadBalanceReference() {
  if (!prefs.getBool(NVS_KEY_VALID, false)) return false;
  balanceReference = prefs.getFloat(NVS_KEY_REF, 0.0f);
  return true;
}

// ============================================================
// TB6612FNG 모터 제어
// ============================================================
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
    // Coast: FALL/DISARM에서 출력 해제
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_PIN, 0);
  }
}

void stopMotorNow() {
  appliedPwm = 0;
  motorMemory = 0.0f;
  wheelAngleTrim = 0.0f;
  applyMotorPwm(0);
}

int applyMinimumPwm(int pwm) {
  if (pwm == 0) return 0;
  if (abs(pwm) < MIN_EFFECTIVE_PWM) {
    return (pwm > 0) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
  }
  return pwm;
}

void setMotorTarget(int targetPwm, bool kick) {
  targetPwm = constrain(targetPwm, -PWM_LIMIT, PWM_LIMIT);

  if (kick) {
    appliedPwm = targetPwm;
  } else if (targetPwm > appliedPwm + PWM_SLEW_PER_LOOP) {
    appliedPwm += PWM_SLEW_PER_LOOP;
  } else if (targetPwm < appliedPwm - PWM_SLEW_PER_LOOP) {
    appliedPwm -= PWM_SLEW_PER_LOOP;
  } else {
    appliedPwm = targetPwm;
  }

  applyMotorPwm(appliedPwm);
}

// ============================================================
// 시리얼 명령
// c + Enter: 현재 edge 평형 자세를 기준각으로 저장
// x + Enter: 저장 기준각 삭제
// r + Enter: motorMemory, wheelAngleTrim 초기화
// ============================================================
void handleSerialCommands() {
  if (!Serial.available()) return;

  char c = Serial.read();

  if (c == 'c' || c == 'C') {
    armed = false;
    armCandidateMs = 0;
    stopMotorNow();
    captureBalanceReference();

    gxFilt = 0.0f;
    gyFilt = 0.0f;
    gzFilt = 0.0f;
    motorMemory = 0.0f;
    wheelAngleTrim = 0.0f;

  } else if (c == 'x' || c == 'X') {
    armed = false;
    stopMotorNow();
    prefs.remove(NVS_KEY_REF);
    prefs.remove(NVS_KEY_VALID);
    Serial.println("Saved reference erased.");

  } else if (c == 'r' || c == 'R') {
    motorMemory = 0.0f;
    wheelAngleTrim = 0.0f;
    Serial.println("Wheel memory and trim reset.");
  }
}

// ============================================================
// setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=== smallcube_v5: 1-Axis Reaction Wheel Balance ===");
  Serial.println("I2C: SDA=GPIO2, SCL=GPIO3 (GPIO8/9 unused)");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Arduino-ESP32 Core 3.x LEDC API
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_PIN, 0);

  prefs.begin(NVS_NAMESPACE, false);
  initSensors();
  calibrateGyro();

  if (loadBalanceReference()) {
    Serial.printf("Loaded saved reference: %.3f deg (%s)\n",
                  balanceReference, USE_PITCH_AXIS ? "PITCH" : "ROLL");
  } else {
    Serial.println("No saved reference. First-time capture required.");
    captureBalanceReference();
  }

  gxFilt = gyFilt = gzFilt = 0.0f;
  motorMemory = 0.0f;
  wheelAngleTrim = 0.0f;
  armed = false;
  armCandidateMs = 0;

  digitalWrite(STBY, HIGH);
  lastLoopUs = micros();

  Serial.println("Commands: c=capture/save, x=erase reference, r=reset wheel memory");
  Serial.println("Ready: hold near reference for 0.5 s to ARM.");
  Serial.println("angle,rate,error,target,pwm,armed,saturated,memory,trim");
}

// ============================================================
// loop
// ============================================================
void loop() {
  handleSerialCommands();

  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) < LOOP_US) return;

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

  // ADXL345 raw -> g
  const float ax = axRaw * 0.00390625f;
  const float ay = ayRaw * 0.00390625f;
  const float az = azRaw * 0.00390625f;

  // 가속도 기반 자세각
  const float rollAcc = atan2f(ay, az) * 180.0f / PI;
  const float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  if (!isfinite(rollAcc) || !isfinite(pitchAcc)) {
    armed = false;
    stopMotorNow();
    return;
  }

  // ITG3200 raw -> deg/s, 부팅 시 측정한 offset 제거
  const float gxDps = ((float)gxRaw - gyroOffsetX) / GYRO_LSB_PER_DPS;
  const float gyDps = ((float)gyRaw - gyroOffsetY) / GYRO_LSB_PER_DPS;
  const float gzDps = ((float)gzRaw - gyroOffsetZ) / GYRO_LSB_PER_DPS;

  // 자이로 LPF
  gxFilt += GYRO_LPF_ALPHA * (gxDps - gxFilt);
  gyFilt += GYRO_LPF_ALPHA * (gyDps - gyFilt);
  gzFilt += GYRO_LPF_ALPHA * (gzDps - gzFilt);

  // 상보필터: 자세각 = 자이로 적분 + 가속도 장기 보정
  roll = COMP_ALPHA * (roll + gxFilt * dt)
       + (1.0f - COMP_ALPHA) * rollAcc;
  pitch = COMP_ALPHA * (pitch + gyFilt * dt)
        + (1.0f - COMP_ALPHA) * pitchAcc;

  const float angle = USE_PITCH_AXIS ? pitch : roll;
  const float rate = USE_PITCH_AXIS ? gyFilt : gxFilt;
  const float error = balanceReference - angle;
  const float absError = fabsf(error);
  const uint32_t nowMs = millis();

  // -------------------- ARM / FALL --------------------
  if (!armed) {
    stopMotorNow();

    if (absError < ARM_ANGLE_DEG) {
      if (armCandidateMs == 0) armCandidateMs = nowMs;

      if (nowMs - armCandidateMs >= ARM_HOLD_MS) {
        armed = true;
        motorMemory = 0.0f;
        wheelAngleTrim = 0.0f;
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

  // -------------------- PD + 엔코더 없는 Mpre 대용 피드백 --------------------
  int targetPwm = 0;
  bool saturated = false;

  if (armed) {
    // 이전 PWM을 부드럽게 누적해 휠 회전 상태의 대용값으로 사용
    motorMemory += MOTOR_MEMORY_ALPHA * ((float)appliedPwm - motorMemory);

    // 한 방향 구동이 오래 지속되면 가상 기준각을 천천히 이동
    wheelAngleTrim += IDRS * CONTROL_SIGN * motorMemory * dt;
    wheelAngleTrim = constrain(wheelAngleTrim,
                               -WHEEL_TRIM_LIMIT_DEG,
                                WHEEL_TRIM_LIMIT_DEG);

    // P: 자세 오차, D: 프레임 회전 감쇠, W: 누적 휠 상태 억제
    float control = CONTROL_SIGN * (
      KP * (error - wheelAngleTrim)
      - KD * rate
      - KW * motorMemory
    );

    if (fabsf(control) >= PWM_LIMIT) saturated = true;

    targetPwm = (int)lroundf(control);
    targetPwm = constrain(targetPwm, -PWM_LIMIT, PWM_LIMIT);

    if (abs(targetPwm) <= PWM_DEADBAND) {
      targetPwm = 0;
    } else {
      targetPwm = applyMinimumPwm(targetPwm);
    }

    const bool kick = (absError >= KICK_ERROR_DEG);
    setMotorTarget(targetPwm, kick);
  }

  // -------------------- 20 Hz CSV 로그 --------------------
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
    Serial.print(saturated ? 1 : 0);
    Serial.print(",");
    Serial.print(motorMemory, 1);
    Serial.print(",");
    Serial.println(wheelAngleTrim, 3);
  }
}
