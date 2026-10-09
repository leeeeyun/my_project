#include <Wire.h>
#include <Preferences.h>
#include <Kalman.h>

// ============================================================
// smallcube_v8_kalman_damping_tuned.ino
// ESP32-C3 SuperMini + GY-85(ADXL345 + ITG3200) + TB6612FNG
// 1축 반작용휠 밸런싱 큐브
//
// v8 변경점 (limit cycle 진동 억제 튜닝)
// 1) KD  16 -> 27   : rate 커질 때 더 강하게 제동 (오버슈트 억제)
// 2) KP  85 -> 68   : P항 과도한 밀어붙임 완화, D항이 감쇠 담당
// 3) PWM_SLEW_PER_LOOP 90 -> 55 : 부호 반전 시 급격한 반대토크 완화
// 4) RATE_LPF_ALPHA는 0.55 유지 (위 3개 먼저 검증 후 필요시 0.7로 조정)
// 5) v7의 Kalman.getAngle() 인자없이 호출하던 버그 수정
//    (lastKalRoll/lastKalPitch로 이전 추정값을 별도 저장)
//
// 시리얼 명령 (줄바꿈 없이 한 글자만 전송해도 됨)
// c : 현재 자세를 목표각으로 저장
// x : 저장 목표각 삭제
// r : wheel memory / trim / bias 초기화
// b : WHEEL_BIAS +5
// B : WHEEL_BIAS -5
// p : 현재 설정 출력
// ============================================================

#define SDA_PIN 2
#define SCL_PIN 3
#define IN1     4
#define IN2     5
#define PWM_PIN 6
#define STBY    7

#define PWM_FREQ 20000
#define PWM_RES  8

#define ADXL345_ADDR 0x53
#define ITG3200_ADDR 0x68

const uint32_t LOOP_US = 5000;  // 200 Hz
const float GYRO_LSB_PER_DPS = 14.375f;
const bool USE_PITCH_AXIS = true;

// -------------------- Kalman 설정 --------------------
const float KALMAN_Q_ANGLE = 0.001f;
const float KALMAN_Q_BIAS  = 0.003f;
const float KALMAN_R_MEASURE = 0.10f;
const float ACCEL_OUTLIER_LIMIT_DEG = 7.0f;

Kalman kalRoll;
Kalman kalPitch;

// 이전 추정 각도 (outlier 제한용, Kalman 라이브러리에는 getAngle() 단독 조회가 없음)
float lastKalRoll = 0.0f;
float lastKalPitch = 0.0f;

// -------------------- 제어 게인 (v8 튜닝) --------------------
float KP = 68.0f;    // 85 -> 68
float KD = 27.0f;    // 16 -> 27
float KW = 0.12f;
float IDRS = 0.0012f;

// D항용 rate 저역통과필터
const float RATE_LPF_ALPHA = 0.55f; // 필요시 0.7까지 올려서 재시도
float rateControl = 0.0f;

// 목표 근처에서도 휠에 유지 토크를 주기 위한 항 (b/B로 조절)
float wheelBias = 0.0f;
const float WHEEL_BIAS_LIMIT = 100.0f;

const int CONTROL_SIGN = -1; // 방향 반대면 +1
const int PWM_LIMIT = 255;
const int PWM_DEADBAND = 0;
const int MIN_EFFECTIVE_PWM = 30;
const int PWM_SLEW_PER_LOOP = 55; // 90 -> 55

const float ARM_ANGLE_DEG = 4.0f;
const float FALL_ANGLE_LEFT_DEG = 15.0f;   // angle > -15: 왼쪽 낙상
const float FALL_ANGLE_RIGHT_DEG = -32.0f; // angle < -32: 오른쪽 낙상
const uint32_t ARM_HOLD_MS = 500;

const float MOTOR_MEMORY_ALPHA = 0.05f;
const float WHEEL_TRIM_LIMIT_DEG = 2.0f;

Preferences prefs;
const char *NVS_NAMESPACE = "balcube";
const char *NVS_KEY_REF = "refAngle";
const char *NVS_KEY_VALID = "refValid";

float gyroOffsetX = 0.0f, gyroOffsetY = 0.0f, gyroOffsetZ = 0.0f;
float rollAcc = 0.0f, pitchAcc = 0.0f;
float roll = 0.0f, pitch = 0.0f;
float balanceReference = 0.0f;

int appliedPwm = 0;
float motorMemory = 0.0f;
float wheelAngleTrim = 0.0f;
bool armed = false;
uint32_t armCandidateMs = 0;
uint32_t lastLoopUs = 0;
uint32_t lastPrintMs = 0;

// -------------------- I2C --------------------
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

// -------------------- 센서 읽기 --------------------
bool readADXL345(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t data[6];
  if (!readBytes(ADXL345_ADDR, 0x32, data, 6)) return false;
  ax = (int16_t)((data[1] << 8) | data[0]);
  ay = (int16_t)((data[3] << 8) | data[2]);
  az = (int16_t)((data[5] << 8) | data[4]);
  return true;
}

bool readITG3200(int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t data[6];
  if (!readBytes(ITG3200_ADDR, 0x1D, data, 6)) return false;
  gx = (int16_t)((data[0] << 8) | data[1]);
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
  Wire.setClock(100000);

  writeRegister(ADXL345_ADDR, 0x31, 0x08);
  writeRegister(ADXL345_ADDR, 0x2C, 0x0B);
  writeRegister(ADXL345_ADDR, 0x2D, 0x08);

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

  Serial.println("Keep cube still for gyro warm-up (2 s).");
  delay(2000);
  Serial.println("Gyro calibration: do not touch cube.");

  for (int i = 0; i < SAMPLES; i++) {
    int16_t gx, gy, gz;
    if (readITG3200(gx, gy, gz)) {
      sumX += gx; sumY += gy; sumZ += gz;
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

// -------------------- 기준각 --------------------
void captureBalanceReference() {
  const int SAMPLES = 500;
  float sumRoll = 0.0f, sumPitch = 0.0f;
  int valid = 0;

  Serial.println("=== CAPTURE MODE ===");
  Serial.println("Hold desired edge pose. Capture starts in 2 seconds.");
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

  kalRoll.setAngle(roll);
  kalPitch.setAngle(pitch);
  lastKalRoll = roll;
  lastKalPitch = pitch;
  rateControl = 0.0f;

  prefs.putFloat(NVS_KEY_REF, balanceReference);
  prefs.putBool(NVS_KEY_VALID, true);
  Serial.printf("Saved reference: %.3f deg (%s)\n", balanceReference, USE_PITCH_AXIS ? "PITCH" : "ROLL");
}

bool loadBalanceReference() {
  if (!prefs.getBool(NVS_KEY_VALID, false)) return false;
  balanceReference = prefs.getFloat(NVS_KEY_REF, 0.0f);
  return true;
}

// -------------------- 모터 --------------------
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
    ledcWrite(PWM_PIN, 0);
  }
}

void stopMotorNow() {
  appliedPwm = 0;
  motorMemory = 0.0f;
  wheelAngleTrim = 0.0f;
  rateControl = 0.0f;
  applyMotorPwm(0);
}

int applyMinimumPwm(int pwm) {
  if (pwm == 0) return 0;
  if (abs(pwm) < MIN_EFFECTIVE_PWM) return (pwm > 0) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
  return pwm;
}

void setMotorTarget(int targetPwm) {
  targetPwm = constrain(targetPwm, -PWM_LIMIT, PWM_LIMIT);
  if (targetPwm > appliedPwm + PWM_SLEW_PER_LOOP) appliedPwm += PWM_SLEW_PER_LOOP;
  else if (targetPwm < appliedPwm - PWM_SLEW_PER_LOOP) appliedPwm -= PWM_SLEW_PER_LOOP;
  else appliedPwm = targetPwm;
  applyMotorPwm(appliedPwm);
}

void printSettings() {
  Serial.printf("SET ref=%.3f KP=%.1f KD=%.1f KW=%.3f bias=%.1f minPWM=%d slew=%d R=%.3f\n",
                balanceReference, KP, KD, KW, wheelBias, MIN_EFFECTIVE_PWM, PWM_SLEW_PER_LOOP, KALMAN_R_MEASURE);
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\r' || c == '\n') return;

  if (c == 'c' || c == 'C') {
    armed = false;
    armCandidateMs = 0;
    stopMotorNow();
    captureBalanceReference();
    Serial.println("Reference capture complete.");
  } else if (c == 'x' || c == 'X') {
    armed = false;
    armCandidateMs = 0;
    stopMotorNow();
    prefs.remove(NVS_KEY_REF);
    prefs.remove(NVS_KEY_VALID);
    balanceReference = 0.0f;
    Serial.println("Reference erased. Target is now 0 deg.");
  } else if (c == 'r' || c == 'R') {
    motorMemory = 0.0f;
    wheelAngleTrim = 0.0f;
    wheelBias = 0.0f;
    Serial.println("Memory, trim, and wheelBias reset.");
  } else if (c == 'b') {
    wheelBias = constrain(wheelBias + 5.0f, -WHEEL_BIAS_LIMIT, WHEEL_BIAS_LIMIT);
    Serial.printf("wheelBias = %.1f\n", wheelBias);
  } else if (c == 'B') {
    wheelBias = constrain(wheelBias - 5.0f, -WHEEL_BIAS_LIMIT, WHEEL_BIAS_LIMIT);
    Serial.printf("wheelBias = %.1f\n", wheelBias);
  } else if (c == 'p' || c == 'P') {
    printSettings();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== smallcube_v8: Kalman + Damping Tuned (KP68/KD27/slew55) ===");
  Serial.println("I2C: SDA=GPIO2, SCL=GPIO3; Motor: IN1=4, IN2=5, PWM=6, STBY=7");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(STBY, LOW);

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_PIN, 0);

  prefs.begin(NVS_NAMESPACE, false);
  initSensors();
  calibrateGyro();

  if (!readAccelAngles(rollAcc, pitchAcc)) {
    Serial.println("FATAL: ADXL345 read failed.");
    while (true) delay(1000);
  }

  roll = rollAcc;
  pitch = pitchAcc;

  kalRoll.setQangle(KALMAN_Q_ANGLE);
  kalRoll.setQbias(KALMAN_Q_BIAS);
  kalRoll.setRmeasure(KALMAN_R_MEASURE);
  kalPitch.setQangle(KALMAN_Q_ANGLE);
  kalPitch.setQbias(KALMAN_Q_BIAS);
  kalPitch.setRmeasure(KALMAN_R_MEASURE);

  kalRoll.setAngle(rollAcc);
  kalPitch.setAngle(pitchAcc);
  lastKalRoll = rollAcc;
  lastKalPitch = pitchAcc;

  if (loadBalanceReference()) {
    Serial.printf("Loaded saved reference: %.3f deg (%s)\n", balanceReference, USE_PITCH_AXIS ? "PITCH" : "ROLL");
  } else {
    balanceReference = 0.0f;
    Serial.println("No saved reference: target is 0 deg.");
  }

  digitalWrite(STBY, HIGH);
  lastLoopUs = micros();
  Serial.println("Commands: c=save, x=erase, r=reset, b=bias+5, B=bias-5, p=print");
  Serial.println("angle,rate,error,target,pwm,armed,saturated,memory,trim,bias");
}

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

  const float gxDps = ((float)gxRaw - gyroOffsetX) / GYRO_LSB_PER_DPS;
  const float gyDps = ((float)gyRaw - gyroOffsetY) / GYRO_LSB_PER_DPS;

  // 가속도계 튐을 그대로 Kalman에 주지 않도록 이전 추정값 기준으로 제한
  float rollMeas = rollAcc;
  float pitchMeas = pitchAcc;
  float rollDiff = rollMeas - lastKalRoll;
  float pitchDiff = pitchMeas - lastKalPitch;

  if (fabsf(rollDiff) > ACCEL_OUTLIER_LIMIT_DEG) {
    rollMeas = lastKalRoll + copysignf(ACCEL_OUTLIER_LIMIT_DEG, rollDiff);
  }
  if (fabsf(pitchDiff) > ACCEL_OUTLIER_LIMIT_DEG) {
    pitchMeas = lastKalPitch + copysignf(ACCEL_OUTLIER_LIMIT_DEG, pitchDiff);
  }

  // 자세 추정: Kalman 라이브러리만 사용
  roll = kalRoll.getAngle(rollMeas, gxDps, dt);
  pitch = kalPitch.getAngle(pitchMeas, gyDps, dt);

  lastKalRoll = roll;
  lastKalPitch = pitch;

  const float angle = USE_PITCH_AXIS ? pitch : roll;
  const float rawRate = USE_PITCH_AXIS ? kalPitch.getRate() : kalRoll.getRate();
  rateControl += RATE_LPF_ALPHA * (rawRate - rateControl); // D항 센서 튐 완화

  const float error = balanceReference - angle;
  const float absError = fabsf(error);
  const uint32_t nowMs = millis();

  if (!armed) {
    stopMotorNow();
    if (absError < ARM_ANGLE_DEG) {
      if (armCandidateMs == 0) armCandidateMs = nowMs;
      if (nowMs - armCandidateMs >= ARM_HOLD_MS) {
        armed = true;
        motorMemory = 0.0f;
        wheelAngleTrim = 0.0f;
        rateControl = 0.0f;
        Serial.println("[STATE] ARMED");
      }
    } else {
      armCandidateMs = 0;
    }
  } else {
    if (angle > -FALL_ANGLE_LEFT_DEG) {
      armed = false;
      armCandidateMs = 0;
      stopMotorNow();
      Serial.println("[STATE] FALL LEFT / DISARM");
    } else if (angle < FALL_ANGLE_RIGHT_DEG) {
      armed = false;
      armCandidateMs = 0;
      stopMotorNow();
      Serial.println("[STATE] FALL RIGHT / DISARM");
    }
  }

  int targetPwm = 0;
  bool saturated = false;

  if (armed) {
    motorMemory += MOTOR_MEMORY_ALPHA * ((float)appliedPwm - motorMemory);

    wheelAngleTrim += IDRS * CONTROL_SIGN * motorMemory * dt;
    wheelAngleTrim = constrain(wheelAngleTrim, -WHEEL_TRIM_LIMIT_DEG, WHEEL_TRIM_LIMIT_DEG);

    // P: 자세 복원, D: 몸체 각속도 감쇠(강화), W: 휠 과도 누적 억제
    float control = CONTROL_SIGN * (
      KP * (error - wheelAngleTrim)
      - KD * rateControl
      - KW * motorMemory
      + wheelBias
    );

    if (fabsf(control) >= PWM_LIMIT) saturated = true;
    targetPwm = constrain((int)lroundf(control), -PWM_LIMIT, PWM_LIMIT);

    if (abs(targetPwm) <= PWM_DEADBAND) targetPwm = 0;
    else targetPwm = applyMinimumPwm(targetPwm);

    setMotorTarget(targetPwm);
  }

  if (nowMs - lastPrintMs >= 50) {
    lastPrintMs = nowMs;
    Serial.print(angle, 2); Serial.print(',');
    Serial.print(rateControl, 2); Serial.print(',');
    Serial.print(error, 2); Serial.print(',');
    Serial.print(targetPwm); Serial.print(',');
    Serial.print(appliedPwm); Serial.print(',');
    Serial.print(armed ? 1 : 0); Serial.print(',');
    Serial.print(saturated ? 1 : 0); Serial.print(',');
    Serial.print(motorMemory, 1); Serial.print(',');
    Serial.print(wheelAngleTrim, 3); Serial.print(',');
    Serial.println(wheelBias, 1);
  }
}