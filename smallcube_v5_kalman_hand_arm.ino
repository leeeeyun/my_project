#include <Wire.h>
#include <Preferences.h>
#include <Kalman.h>

// ============================================================
// smallcube_v5_kalman_hand_arm.ino
// ESP32-C3 SuperMini + GY-85(ADXL345 + ITG3200) + TB6612FNG
// 1축 반작용휠 밸런싱 큐브
//
// 동작: 전원을 켠 뒤 큐브를 손으로 목표 엣지 자세 근처에 세우고
//       약 0.5초 유지하면 자동 ARM되어 반작용휠 제어를 시작한다.
//
// 시리얼 명령:
//   c + Enter : 현재 손으로 잡은 자세를 기준각으로 저장
//   x + Enter : 저장 기준각 삭제 (0 deg 기준으로 복귀)
//   r + Enter : 제어 누적값 초기화
// ============================================================

// -------------------- 핀 정의: smallcube_v5 그대로 유지 --------------------
#define SDA_PIN 2
#define SCL_PIN 3
#define IN1     4
#define IN2     5
#define PWM_PIN 6
#define STBY    7

#define PWM_FREQ 20000
#define PWM_RES  8

// -------------------- GY-85 I2C 주소 --------------------
#define ADXL345_ADDR 0x53
#define ITG3200_ADDR 0x68

// -------------------- 제어 주기 --------------------
const uint32_t LOOP_US = 5000;  // 200 Hz

// -------------------- Kalman / 자이로 설정 --------------------
const float GYRO_LSB_PER_DPS = 14.375f;  // ITG3200 ±2000 dps

// true: pitch(ADXL X, ITG Y), false: roll(ADXL Y, ITG X)
// 휠 축과 수직인 큐브 기울기 축에 맞춰 선택한다.
const bool USE_PITCH_AXIS = true;

// Kalman 라이브러리 기본 파라미터는 유지한다.
// 필요하면 아래 3개 값을 조절할 수 있다.
const float KALMAN_Q_ANGLE = 0.001f;
const float KALMAN_Q_BIAS  = 0.003f;
const float KALMAN_R_MEAS  = 0.03f;

// -------------------- 제어 게인: 최초 안전값 --------------------
float KP = 80.0f;     // ↓ P 게인 감소 (과제어 방지)
float KD = 12.0f;     // ↑ D 게인 증가 (진동 감쇠)
float KW = 0.1f;      // ↑ 휠 메모리 억제 강화
float IDRS = 0.005f;  // ↓ 적분 속도 감소 (기준각 폭주 방지)

const int CONTROL_SIGN = 1;   // 반대로 넘어지면 -1로 변경
const int PWM_LIMIT = 240;
const int PWM_DEADBAND = 2;
const int MIN_EFFECTIVE_PWM = 18;
const int PWM_SLEW_PER_LOOP = 130;

// ARM: 손으로 기준 자세 근처에 세우면 자동 구동 시작
const float ARM_ANGLE_DEG = 4.0f;
const float FALL_ANGLE_DEG = 28.0f;
const uint32_t ARM_HOLD_MS = 500;

// 엔코더가 없으므로 이전 PWM을 휠 상태의 근사값으로 사용한다.
const float MOTOR_MEMORY_ALPHA = 0.05f;
const float WHEEL_TRIM_LIMIT_DEG = 2.0f;

// -------------------- NVS 기준각 저장 --------------------
Preferences prefs;
const char *NVS_NAMESPACE = "balcube";
const char *NVS_KEY_REF = "refAngle";
const char *NVS_KEY_VALID = "refValid";

// -------------------- 센서 상태 --------------------
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

float rollAcc = 0.0f;
float pitchAcc = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float balanceReference = 0.0f;

Kalman kalmanRoll;
Kalman kalmanPitch;
float angle = 0.0f;
float rate = 0.0f;

// -------------------- 제어 상태 --------------------
int appliedPwm = 0;
float motorMemory = 0.0f;
float wheelAngleTrim = 0.0f;
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

bool readBytes(uint8_t address, uint8_t startReg, uint8_t *buffer, uint8_t length) {
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

// SDA가 LOW에 고정된 경우 SCL 펄스로 I2C 버스를 복구한다.
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
// GY-85 읽기
// ============================================================
bool readADXL345(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t data[6];
  if (!readBytes(ADXL345_ADDR, 0x32, data, 6)) return false;

  ax = (int16_t)((data[1] << 8) | data[0]);  // ADXL345: little-endian
  ay = (int16_t)((data[3] << 8) | data[2]);
  az = (int16_t)((data[5] << 8) | data[4]);
  return true;
}

bool readITG3200(int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t data[6];
  if (!readBytes(ITG3200_ADDR, 0x1D, data, 6)) return false;

  gx = (int16_t)((data[0] << 8) | data[1]);  // ITG3200: big-endian
  gy = (int16_t)((data[2] << 8) | data[3]);
  gz = (int16_t)((data[4] << 8) | data[5]);
  return true;
}

bool readAccelAngles(float &outRoll, float &outPitch) {
  int16_t axRaw, ayRaw, azRaw;
  if (!readADXL345(axRaw, ayRaw, azRaw)) return false;

  const float ax = axRaw * 0.00390625f;
  const float ay = ayRaw * 0.00390625f;
  const float az = azRaw * 0.00390625f;

  outRoll = atan2f(ay, az) * 180.0f / PI;
  outPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  return isfinite(outRoll) && isfinite(outPitch);
}

void initSensors() {
  recoverI2CBus();
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // GY-85 배선 환경 안정성 우선

  // ADXL345: full-resolution, ±2g, 200 Hz, measurement mode
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

  Serial.println("Keep cube still on a flat surface for gyro warm-up (2 s).");
  delay(2000);
  Serial.println("Gyro calibration: do not touch cube.");

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
    Serial.println("FATAL: ITG3200 read failed.");
    while (true) delay(1000);
  }

  gyroOffsetX = (float)sumX / valid;
  gyroOffsetY = (float)sumY / valid;
  gyroOffsetZ = (float)sumZ / valid;
  Serial.printf("Gyro offsets: %.1f, %.1f, %.1f\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
}

// ============================================================
// 기준각: 선택 기능
// 저장값이 없으면 0 deg 기준으로 자동 ARM만 사용한다.
// c + Enter를 쓰면 현재 손으로 잡은 목표 자세가 기준각으로 저장된다.
// ============================================================
void captureBalanceReference() {
  const int SAMPLES = 500;
  float sumRoll = 0.0f;
  float sumPitch = 0.0f;
  int valid = 0;

  Serial.println("=== CAPTURE MODE ===");
  Serial.println("Hold the desired edge pose. Capture starts in 2 seconds.");
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
  Serial.printf("Saved reference: %.3f deg (%s)\n", balanceReference,
                USE_PITCH_AXIS ? "PITCH" : "ROLL");
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
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_PIN, 0);  // Coast
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
  if (abs(pwm) < MIN_EFFECTIVE_PWM) return (pwm > 0) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
  return pwm;
}

void setMotorTarget(int targetPwm) {
  targetPwm = constrain(targetPwm, -PWM_LIMIT, PWM_LIMIT);

  // 급격한 반전 전류를 줄이기 위한 PWM slew 제한
  if (targetPwm > appliedPwm + PWM_SLEW_PER_LOOP) {
    appliedPwm += PWM_SLEW_PER_LOOP;
  } else if (targetPwm < appliedPwm - PWM_SLEW_PER_LOOP) {
    appliedPwm -= PWM_SLEW_PER_LOOP;
  } else {
    appliedPwm = targetPwm;
  }

  applyMotorPwm(appliedPwm);
}

// ============================================================
// 시리얼 명령: c, x, r 뒤 Enter
// ============================================================
void handleSerialCommands() {
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == '\r' || c == '\n') return;

  if (c == 'c' || c == 'C') {
    armed = false;
    armCandidateMs = 0;
    stopMotorNow();
    captureBalanceReference();

    // 캡처 이후 칼만 필터도 현재 자세에서 다시 시작
    kalmanRoll.setAngle(roll);
    kalmanPitch.setAngle(pitch);
    Serial.println("Reference capture complete.");

  } else if (c == 'x' || c == 'X') {
    armed = false;
    stopMotorNow();
    prefs.remove(NVS_KEY_REF);
    prefs.remove(NVS_KEY_VALID);
    balanceReference = 0.0f;
    Serial.println("Reference erased. Target is now 0 deg.");

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

  Serial.println("\n=== smallcube_v5: Kalman Hand-ARM Balance ===");
  Serial.println("I2C: SDA=GPIO2, SCL=GPIO3; Motor: IN1=4, IN2=5, PWM=6, STBY=7");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(STBY, LOW);  // 센서/보정 중 휠 정지

  // Arduino-ESP32 Core 3.x LEDC API
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_PIN, 0);

  prefs.begin(NVS_NAMESPACE, false);
  initSensors();
  calibrateGyro();

  // 가속도계 현재값으로 칼만 필터 초기화
  if (!readAccelAngles(rollAcc, pitchAcc)) {
    Serial.println("FATAL: ADXL345 read failed.");
    while (true) delay(1000);
  }

  roll = rollAcc;
  pitch = pitchAcc;
  kalmanRoll.setQangle(KALMAN_Q_ANGLE);
  kalmanRoll.setQbias(KALMAN_Q_BIAS);
  kalmanRoll.setRmeasure(KALMAN_R_MEAS);
  kalmanPitch.setQangle(KALMAN_Q_ANGLE);
  kalmanPitch.setQbias(KALMAN_Q_BIAS);
  kalmanPitch.setRmeasure(KALMAN_R_MEAS);
  kalmanRoll.setAngle(roll);
  kalmanPitch.setAngle(pitch);

  // 저장된 기준각은 선택 사항이다. 없으면 0 deg를 목표로 손으로 세우면 된다.
  if (loadBalanceReference()) {
    Serial.printf("Loaded saved reference: %.3f deg (%s)\n", balanceReference,
                  USE_PITCH_AXIS ? "PITCH" : "ROLL");
  } else {
    balanceReference = 0.0f;
    Serial.println("No saved reference: target is 0 deg. Hold cube upright to ARM.");
  }

  digitalWrite(STBY, HIGH);
  lastLoopUs = micros();

  Serial.println("Commands: c=save current pose, x=erase reference, r=reset memory");
  Serial.println("Hand-ARM: hold within ARM_ANGLE_DEG for ARM_HOLD_MS to start.");
  Serial.println("angle,rate,error,target,pwm,armed,saturated,memory,trim");
}

// ============================================================
// loop: IMU -> Kalman -> ARM/FALL -> PD + wheel memory -> TB6612
// ============================================================
void loop() {
  handleSerialCommands();

  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) < LOOP_US) return;

  float dt = (nowUs - lastLoopUs) * 0.000001f;
  lastLoopUs = nowUs;
  dt = constrain(dt, 0.001f, 0.020f);

  int16_t gxRaw, gyRaw, gzRaw;
  const bool accOK = readAccelAngles(rollAcc, pitchAcc);
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

  // ITG3200 raw -> deg/s, 부팅 시 정지 보정값 제거
  const float gxDps = ((float)gxRaw - gyroOffsetX) / GYRO_LSB_PER_DPS;
  const float gyDps = ((float)gyRaw - gyroOffsetY) / GYRO_LSB_PER_DPS;

  // Kalman: 가속도 각도로 장기 드리프트를 보정하고 자이로로 빠른 움직임을 추정
  roll = kalmanRoll.getAngle(rollAcc, gxDps, dt);
  pitch = kalmanPitch.getAngle(pitchAcc, gyDps, dt);

  angle = USE_PITCH_AXIS ? pitch : roll;
  rate = USE_PITCH_AXIS ? kalmanPitch.getRate() : kalmanRoll.getRate();
  const float error = balanceReference - angle;
  const float absError = fabsf(error);
  const uint32_t nowMs = millis();

  // 손으로 기준각 근처에 세운 상태를 0.5초 유지하면 자동 ARM
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

  // 크게 넘어지면 즉시 모터를 coast 상태로 만들고 다시 손으로 세우기를 기다림
  } else if (absError > FALL_ANGLE_DEG) {
    armed = false;
    armCandidateMs = 0;
    stopMotorNow();
    Serial.println("[STATE] FALL / DISARM");
  }

  int targetPwm = 0;
  bool saturated = false;

  if (armed) {
    // 엔코더가 없으므로 지난 PWM의 저역통과값을 휠 속도/누적 상태 근사값으로 사용
    motorMemory += MOTOR_MEMORY_ALPHA * ((float)appliedPwm - motorMemory);

    // 지속 편향을 줄이기 위해 기준각을 최대 ±2 deg 안에서 천천히 보정
    wheelAngleTrim += IDRS * CONTROL_SIGN * motorMemory * dt;
    wheelAngleTrim = constrain(wheelAngleTrim, -WHEEL_TRIM_LIMIT_DEG, WHEEL_TRIM_LIMIT_DEG);

    // P: 기울기 복원, D: 몸체 회전 감쇠, W: 휠 과도 누적 억제
    float control = CONTROL_SIGN * (
      KP * (error - wheelAngleTrim)
      - KD * rate
      - KW * motorMemory
    );

    if (fabsf(control) >= PWM_LIMIT) saturated = true;
    targetPwm = constrain((int)lroundf(control), -PWM_LIMIT, PWM_LIMIT);

    if (abs(targetPwm) <= PWM_DEADBAND) {
      targetPwm = 0;
    } else {
      targetPwm = applyMinimumPwm(targetPwm);
    }

    setMotorTarget(targetPwm);
  }

  // 시리얼 플로터/CSV용 20 Hz 로그
  if (nowMs - lastPrintMs >= 50) {
    lastPrintMs = nowMs;
    Serial.print(angle, 2); Serial.print(',');
    Serial.print(rate, 2); Serial.print(',');
    Serial.print(error, 2); Serial.print(',');
    Serial.print(targetPwm); Serial.print(',');
    Serial.print(appliedPwm); Serial.print(',');
    Serial.print(armed ? 1 : 0); Serial.print(',');
    Serial.print(saturated ? 1 : 0); Serial.print(',');
    Serial.print(motorMemory, 1); Serial.print(',');
    Serial.println(wheelAngleTrim, 3);
  }
}
