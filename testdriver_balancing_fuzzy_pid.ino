/*
 * ================================================================
 *  ESP32 Self-Balancing Robot - Fuzzy-PID Controller
 * ================================================================
 * 
 * Metode: FUZZY-PID (Fuzzy tuning PID parameters)
 *   Fuzzy Logic → adjust Kp, Ki, Kd secara dinamis
 *   PID Controller → output ke motor
 * 
 * Flow:
 *   MPU6050 → error & d_error → FUZZY → ΔKp, ΔKi, ΔKd
 *   → PID(Kp+ΔKp, Ki+ΔKi, Kd+ΔKd) → Motor L298N
 * 
 * Komponen:
 *   - ESP32 Dev Module
 *   - MPU6050 (Gyro + Accelerometer)
 *   - L298N Motor Driver
 *   - 2x DC Motor + Roda
 *   - Battery 7-12V
 * 
 * Wiring ESP32 → MPU6050 (I2C):
 * ┌──────────┬──────────┐
 * │  ESP32   │  MPU6050 │
 * ├──────────┼──────────┤
 * │  GPIO 21 │  SDA     │
 * │  GPIO 22 │  SCL     │
 * │  3.3V    │  VCC     │
 * │  GND     │  GND     │
 * └──────────┴──────────┘
 * 
 * Wiring ESP32 → L298N:
 * ┌──────────┬──────────┐
 * │  ESP32   │  L298N   │
 * ├──────────┼──────────┤
 * │  GPIO 27 │  IN1     │  Motor A
 * │  GPIO 26 │  IN2     │  Motor A
 * │  GPIO 14 │  ENA     │  Motor A PWM
 * │  GPIO 25 │  IN3     │  Motor B
 * │  GPIO 33 │  IN4     │  Motor B
 * │  GPIO 32 │  ENB     │  Motor B PWM
 * │  GND     │  GND     │
 * └──────────┴──────────┘
 * 
 * Serial Commands (115200 baud):
 *   't' = Toggle balancing ON/OFF
 *   'i' = Info (angle, PID values, Fuzzy output)
 *   'c' = Calibrate MPU6050 (taruh robot tegak dulu!)
 *   'p' = Print current PID parameters
 *   '+' = Tambah setpoint +1°
 *   '-' = Kurang setpoint -1°
 *   '0' = Reset setpoint ke 0°
 *   'd' = Debug mode (print continuous)
 *   's' = Stop motor
 * 
 * by Bara & Lihana
 */

#include <Wire.h>
#include <math.h>

// ========== PIN DEFINITIONS ==========
// Motor A (kiri)
#define IN1   27
#define IN2   26
#define ENA   14

// Motor B (kanan)
#define IN3   25
#define IN4   33
#define ENB   32

// MPU6050 I2C
#define SDA_PIN  21
#define SCL_PIN  22
#define MPU_ADDR 0x68

// ========== PWM CONFIG ==========
#define PWM_FREQ       5000
#define PWM_RESOLUTION 8      // 0-255

// ========== TIMING ==========
#define SAMPLE_TIME    5      // 5ms = 200Hz loop rate
#define PRINT_INTERVAL 200    // Print setiap 200ms

// ========== PID BASE VALUES ==========
// Nilai dasar PID (akan di-adjust oleh Fuzzy)
float Kp_base = 25.0;
float Ki_base = 0.5;
float Kd_base = 15.0;

// PID aktif (base + delta dari Fuzzy)
float Kp = 25.0;
float Ki = 0.5;
float Kd = 15.0;

// Fuzzy delta output
float dKp = 0.0;
float dKi = 0.0;
float dKd = 0.0;

// PID state
float setpoint = 0.0;     // Target angle (tegak = 0°)
float error = 0.0;
float prev_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float pid_output = 0.0;

// Integral windup limit
#define INTEGRAL_LIMIT 100.0

// Motor dead zone (minimum PWM buat motor muter)
#define MOTOR_DEADZONE 30

// Max tilt sebelum cutoff (safety)
#define MAX_TILT_ANGLE 45.0

// ========== MPU6050 VARIABLES ==========
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float gyroX_offset = 0.0;
float gyroY_offset = 0.0;
float gyroZ_offset = 0.0;
float accAngle = 0.0;       // Angle dari accelerometer
float gyroAngle = 0.0;      // Angle dari gyroscope
float currentAngle = 0.0;   // Fused angle (complementary filter)
float alpha = 0.98;         // Complementary filter coefficient

// ========== STATE ==========
bool balancing = false;
bool debugMode = false;
unsigned long lastTime = 0;
unsigned long lastPrint = 0;
float dt = 0.005;           // Delta time in seconds

// ================================================================
//  FUZZY LOGIC - Membership Functions & Rule Base
// ================================================================
// 
// Input 1: error (e)      → NB, NM, NS, ZE, PS, PM, PB
// Input 2: d_error (de)   → NB, NM, NS, ZE, PS, PM, PB
// Output:  ΔKp, ΔKi, ΔKd  → NB, NM, NS, ZE, PS, PM, PB
//
// 7 membership functions × 7 = 49 rules
// ================================================================

// Fuzzy output singletons
#define F_NB  -3.0
#define F_NM  -2.0
#define F_NS  -1.0
#define F_ZE   0.0
#define F_PS   1.0
#define F_PM   2.0
#define F_PB   3.0

// Fuzzy universe boundaries
#define E_MAX   30.0    // Max error (degrees)
#define DE_MAX  50.0    // Max d_error (degrees/sec)

// ========== FUZZY MEMBERSHIP FUNCTIONS ==========

// Triangular membership function
float trimf(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0;
  if (x <= b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

// Left shoulder membership
float left_mf(float x, float a, float b) {
  if (x <= a) return 1.0;
  if (x >= b) return 0.0;
  return (b - x) / (b - a);
}

// Right shoulder membership
float right_mf(float x, float a, float b) {
  if (x <= a) return 0.0;
  if (x >= b) return 1.0;
  return (x - a) / (b - a);
}

// Fuzzify input ke 7 membership values
void fuzzify(float input, float max_val, float membership[7]) {
  // Normalize ke range [-1, 1]
  float x = constrain(input / max_val, -1.0, 1.0);
  
  // NB: left shoulder [-1.0, -0.67]
  membership[0] = left_mf(x, -1.0, -0.67);
  // NM: triangle [-1.0, -0.67, -0.33]
  membership[1] = trimf(x, -1.0, -0.67, -0.33);
  // NS: triangle [-0.67, -0.33, 0.0]
  membership[2] = trimf(x, -0.67, -0.33, 0.0);
  // ZE: triangle [-0.33, 0.0, 0.33]
  membership[3] = trimf(x, -0.33, 0.0, 0.33);
  // PS: triangle [0.0, 0.33, 0.67]
  membership[4] = trimf(x, 0.0, 0.33, 0.67);
  // PM: triangle [0.33, 0.67, 1.0]
  membership[5] = trimf(x, 0.33, 0.67, 1.0);
  // PB: right shoulder [0.67, 1.0]
  membership[6] = right_mf(x, 0.67, 1.0);
}

// ========== FUZZY RULE BASE ==========
// Rule table for ΔKp (49 rules: e × de)
// Rows: error (NB→PB), Cols: d_error (NB→PB)
const float rules_dKp[7][7] = {
  // de:  NB    NM    NS    ZE    PS    PM    PB
  /*NB*/ {F_PB, F_PB, F_PM, F_PM, F_PS, F_ZE, F_ZE},
  /*NM*/ {F_PB, F_PB, F_PM, F_PS, F_PS, F_ZE, F_NS},
  /*NS*/ {F_PM, F_PM, F_PM, F_PS, F_ZE, F_NS, F_NS},
  /*ZE*/ {F_PM, F_PM, F_PS, F_ZE, F_NS, F_NM, F_NM},
  /*PS*/ {F_PS, F_PS, F_ZE, F_NS, F_NS, F_NM, F_NM},
  /*PM*/ {F_PS, F_ZE, F_NS, F_NM, F_NM, F_NM, F_NB},
  /*PB*/ {F_ZE, F_ZE, F_NM, F_NM, F_NM, F_NB, F_NB}
};

// Rule table for ΔKi
const float rules_dKi[7][7] = {
  // de:  NB    NM    NS    ZE    PS    PM    PB
  /*NB*/ {F_NB, F_NB, F_NM, F_NM, F_NS, F_ZE, F_ZE},
  /*NM*/ {F_NB, F_NB, F_NM, F_NS, F_NS, F_ZE, F_ZE},
  /*NS*/ {F_NB, F_NM, F_NS, F_NS, F_ZE, F_PS, F_PS},
  /*ZE*/ {F_NM, F_NM, F_NS, F_ZE, F_PS, F_PM, F_PM},
  /*PS*/ {F_NM, F_NS, F_ZE, F_PS, F_PS, F_PM, F_PB},
  /*PM*/ {F_ZE, F_ZE, F_PS, F_PS, F_PM, F_PB, F_PB},
  /*PB*/ {F_ZE, F_ZE, F_PS, F_PM, F_PM, F_PB, F_PB}
};

// Rule table for ΔKd
const float rules_dKd[7][7] = {
  // de:  NB    NM    NS    ZE    PS    PM    PB
  /*NB*/ {F_PS, F_NS, F_NB, F_NB, F_NB, F_NM, F_PS},
  /*NM*/ {F_PS, F_NS, F_NB, F_NM, F_NM, F_NS, F_ZE},
  /*NS*/ {F_ZE, F_NS, F_NM, F_NM, F_NS, F_NS, F_ZE},
  /*ZE*/ {F_ZE, F_NS, F_NS, F_NS, F_NS, F_NS, F_ZE},
  /*PS*/ {F_ZE, F_ZE, F_ZE, F_ZE, F_ZE, F_ZE, F_ZE},
  /*PM*/ {F_NB, F_NS, F_NS, F_NS, F_ZE, F_PS, F_PB},
  /*PB*/ {F_NB, F_NM, F_NM, F_NS, F_ZE, F_PS, F_PB}
};

// ========== FUZZY INFERENCE (Mamdani + Weighted Average) ==========
float fuzzyInference(float e_membership[7], float de_membership[7], const float rules[7][7]) {
  float numerator = 0.0;
  float denominator = 0.0;
  
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 7; j++) {
      // AND operator = minimum
      float firing_strength = min(e_membership[i], de_membership[j]);
      
      if (firing_strength > 0.001) {  // Skip negligible rules
        float rule_output = rules[i][j];
        numerator += firing_strength * rule_output;
        denominator += firing_strength;
      }
    }
  }
  
  if (denominator < 0.001) return 0.0;
  return numerator / denominator;
}

// ========== FUZZY-PID COMPUTE ==========
// Scale factors untuk fuzzy output → delta PID
#define SCALE_KP  5.0    // ΔKp range: ±15
#define SCALE_KI  0.2    // ΔKi range: ±0.6
#define SCALE_KD  3.0    // ΔKd range: ±9

void fuzzyPIDCompute(float e, float de) {
  // 1. Fuzzify inputs
  float e_mf[7], de_mf[7];
  fuzzify(e, E_MAX, e_mf);
  fuzzify(de, DE_MAX, de_mf);
  
  // 2. Fuzzy inference → defuzzify
  float raw_dKp = fuzzyInference(e_mf, de_mf, rules_dKp);
  float raw_dKi = fuzzyInference(e_mf, de_mf, rules_dKi);
  float raw_dKd = fuzzyInference(e_mf, de_mf, rules_dKd);
  
  // 3. Scale fuzzy output ke delta PID
  dKp = raw_dKp * SCALE_KP;
  dKi = raw_dKi * SCALE_KI;
  dKd = raw_dKd * SCALE_KD;
  
  // 4. Update PID parameters (base + delta)
  Kp = max(0.0f, Kp_base + dKp);
  Ki = max(0.0f, Ki_base + dKi);
  Kd = max(0.0f, Kd_base + dKd);
}

// ================================================================
//  MPU6050 FUNCTIONS
// ================================================================

void mpuInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400kHz I2C
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // Wake up
  Wire.endTransmission(true);
  delay(100);
  
  // Set accelerometer range ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG
  Wire.write(0x00);  // ±2g
  Wire.endTransmission(true);
  
  // Set gyroscope range ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG
  Wire.write(0x00);  // ±250°/s
  Wire.endTransmission(true);
  
  // Set DLPF (Digital Low Pass Filter) ~44Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);  // CONFIG
  Wire.write(0x03);  // DLPF_CFG = 3
  Wire.endTransmission(true);
  
  Serial.println("  ✅ MPU6050 initialized");
}

void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Start register ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  // Raw values
  int16_t rawAccX  = Wire.read() << 8 | Wire.read();
  int16_t rawAccY  = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ  = Wire.read() << 8 | Wire.read();
  int16_t rawTemp  = Wire.read() << 8 | Wire.read();  // Skip temp
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();
  
  // Convert to physical units
  accX = rawAccX / 16384.0;   // ±2g → g
  accY = rawAccY / 16384.0;
  accZ = rawAccZ / 16384.0;
  
  gyroX = (rawGyroX / 131.0) - gyroX_offset;  // ±250°/s → °/s
  gyroY = (rawGyroY / 131.0) - gyroY_offset;
  gyroZ = (rawGyroZ / 131.0) - gyroZ_offset;
}

void mpuCalibrate() {
  Serial.println("  ⏳ Calibrating MPU6050... JANGAN GERAK!");
  
  float sumGX = 0, sumGY = 0, sumGZ = 0;
  int samples = 500;
  
  for (int i = 0; i < samples; i++) {
    mpuRead();
    sumGX += (gyroX + gyroX_offset);  // Raw tanpa offset
    sumGY += (gyroY + gyroY_offset);
    sumGZ += (gyroZ + gyroZ_offset);
    delay(2);
  }
  
  gyroX_offset = sumGX / samples;
  gyroY_offset = sumGY / samples;
  gyroZ_offset = sumGZ / samples;
  
  // Initial angle dari accelerometer
  mpuRead();
  currentAngle = atan2(accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  gyroAngle = currentAngle;
  
  Serial.print("  ✅ Calibration done! Offset: ");
  Serial.print(gyroX_offset, 2);
  Serial.print(", ");
  Serial.print(gyroY_offset, 2);
  Serial.print(", ");
  Serial.println(gyroZ_offset, 2);
  Serial.print("  📐 Initial angle: ");
  Serial.print(currentAngle, 1);
  Serial.println("°");
}

float getAngle() {
  mpuRead();
  
  // Accelerometer angle (pitch)
  accAngle = atan2(accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  
  // Gyroscope integration
  gyroAngle += gyroY * dt;
  
  // Complementary filter
  currentAngle = alpha * (currentAngle + gyroY * dt) + (1.0 - alpha) * accAngle;
  
  return currentAngle;
}

// ================================================================
//  MOTOR CONTROL
// ================================================================

void motorA_drive(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  ledcWrite(ENA, constrain(speed, 0, 255));
}

void motorB_drive(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    speed = -speed;
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  ledcWrite(ENB, constrain(speed, 0, 255));
}

void motorsStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

void motorsDrive(float output) {
  int pwm = (int)output;
  
  // Add dead zone compensation
  if (pwm > 0) pwm += MOTOR_DEADZONE;
  else if (pwm < 0) pwm -= MOTOR_DEADZONE;
  
  // Clamp
  pwm = constrain(pwm, -255, 255);
  
  // Drive both motors sama arah (balancing)
  motorA_drive(pwm);
  motorB_drive(pwm);
}

// ================================================================
//  MAIN SETUP
// ================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println();
  Serial.println("╔══════════════════════════════════════════════╗");
  Serial.println("║  ESP32 Self-Balancing Robot                  ║");
  Serial.println("║  Controller: FUZZY-PID                       ║");
  Serial.println("║  by Bara & Lihana                            ║");
  Serial.println("╚══════════════════════════════════════════════╝");
  Serial.println();
  
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // PWM setup (ESP32 Arduino Core v3.x)
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  
  motorsStop();
  
  // MPU6050
  Serial.println("┌─── INITIALIZING ───┐");
  mpuInit();
  mpuCalibrate();
  Serial.println("└────────────────────┘");
  Serial.println();
  
  // Print PID info
  Serial.println("┌─── FUZZY-PID CONFIG ───┐");
  Serial.print("│ Kp base: ");
  Serial.println(Kp_base);
  Serial.print("│ Ki base: ");
  Serial.println(Ki_base);
  Serial.print("│ Kd base: ");
  Serial.println(Kd_base);
  Serial.print("│ Setpoint: ");
  Serial.print(setpoint);
  Serial.println("°");
  Serial.print("│ Sample time: ");
  Serial.print(SAMPLE_TIME);
  Serial.println("ms");
  Serial.print("│ Max tilt: ±");
  Serial.print(MAX_TILT_ANGLE);
  Serial.println("°");
  Serial.println("└────────────────────────┘");
  Serial.println();
  
  // Menu
  Serial.println("┌─────────── MENU ───────────┐");
  Serial.println("│ 't' = Toggle balancing      │");
  Serial.println("│ 'i' = Info status           │");
  Serial.println("│ 'c' = Calibrate MPU6050     │");
  Serial.println("│ 'p' = Print PID params      │");
  Serial.println("│ '+' = Setpoint +1°          │");
  Serial.println("│ '-' = Setpoint -1°          │");
  Serial.println("│ '0' = Reset setpoint 0°     │");
  Serial.println("│ 'd' = Debug mode toggle     │");
  Serial.println("│ 's' = Stop motor            │");
  Serial.println("└─────────────────────────────┘");
  Serial.println();
  Serial.println(">> Ketik 't' untuk mulai balancing...");
  Serial.println();
  
  lastTime = millis();
}

// ================================================================
//  MAIN LOOP
// ================================================================

void loop() {
  unsigned long now = millis();
  
  // Fixed sample rate
  if (now - lastTime >= SAMPLE_TIME) {
    dt = (now - lastTime) / 1000.0;
    lastTime = now;
    
    // 1. Read angle
    float angle = getAngle();
    
    if (balancing) {
      // Safety cutoff
      if (abs(angle - setpoint) > MAX_TILT_ANGLE) {
        motorsStop();
        balancing = false;
        Serial.println();
        Serial.println("🚨 TILT > 45° — Motor OFF! (safety cutoff)");
        Serial.println("   Tegakkan robot, ketik 't' untuk restart.");
        Serial.println();
        integral = 0;
        return;
      }
      
      // 2. Calculate error & derivative
      error = angle - setpoint;
      derivative = (error - prev_error) / dt;
      
      // 3. FUZZY: adjust PID parameters
      fuzzyPIDCompute(error, derivative);
      
      // 4. PID compute
      integral += error * dt;
      integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
      
      pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
      pid_output = constrain(pid_output, -255.0, 255.0);
      
      // 5. Drive motors
      motorsDrive(pid_output);
      
      // 6. Save previous error
      prev_error = error;
      
      // Debug print
      if (debugMode && (now - lastPrint >= PRINT_INTERVAL)) {
        lastPrint = now;
        Serial.print("A:");
        Serial.print(angle, 1);
        Serial.print("° E:");
        Serial.print(error, 1);
        Serial.print(" Kp:");
        Serial.print(Kp, 1);
        Serial.print(" Ki:");
        Serial.print(Ki, 2);
        Serial.print(" Kd:");
        Serial.print(Kd, 1);
        Serial.print(" PID:");
        Serial.print(pid_output, 0);
        Serial.print(" dKp:");
        Serial.print(dKp, 1);
        Serial.print(" dKi:");
        Serial.print(dKi, 2);
        Serial.print(" dKd:");
        Serial.println(dKd, 1);
      }
    }
  }
  
  // Serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();
    
    switch (cmd) {
      case 't':
      case 'T':
        balancing = !balancing;
        if (balancing) {
          integral = 0;
          prev_error = 0;
          Serial.println("▶️  Balancing ON!");
        } else {
          motorsStop();
          Serial.println("⏹️  Balancing OFF");
        }
        break;
        
      case 'i':
      case 'I':
        Serial.println();
        Serial.println("╔══════════════════════════════════════╗");
        Serial.println("║          STATUS INFO                 ║");
        Serial.println("╠══════════════════════════════════════╣");
        Serial.print("║  Angle     : ");
        Serial.print(currentAngle, 2);
        Serial.println("°              ║");
        Serial.print("║  Setpoint  : ");
        Serial.print(setpoint, 1);
        Serial.println("°               ║");
        Serial.print("║  Error     : ");
        Serial.print(error, 2);
        Serial.println("°              ║");
        Serial.print("║  Balancing : ");
        Serial.println(balancing ? "ON  ▶️          ║" : "OFF ⏹️          ║");
        Serial.print("║  PID Out   : ");
        Serial.print(pid_output, 1);
        Serial.println("                ║");
        Serial.println("║─── Fuzzy-PID Parameters ────────────║");
        Serial.print("║  Kp = ");
        Serial.print(Kp_base, 1);
        Serial.print(" + (");
        Serial.print(dKp, 2);
        Serial.print(") = ");
        Serial.println(Kp, 2);
        Serial.print("║  Ki = ");
        Serial.print(Ki_base, 2);
        Serial.print(" + (");
        Serial.print(dKi, 3);
        Serial.print(") = ");
        Serial.println(Ki, 3);
        Serial.print("║  Kd = ");
        Serial.print(Kd_base, 1);
        Serial.print(" + (");
        Serial.print(dKd, 2);
        Serial.print(") = ");
        Serial.println(Kd, 2);
        Serial.println("╚══════════════════════════════════════╝");
        Serial.println();
        break;
        
      case 'c':
      case 'C':
        balancing = false;
        motorsStop();
        Serial.println("⏳ Re-calibrating... TARUH ROBOT TEGAK!");
        delay(2000);
        mpuCalibrate();
        integral = 0;
        prev_error = 0;
        break;
        
      case 'p':
      case 'P':
        Serial.println();
        Serial.println("┌─── FUZZY-PID PARAMETERS ───┐");
        Serial.print("│ Kp_base = ");
        Serial.println(Kp_base);
        Serial.print("│ Ki_base = ");
        Serial.println(Ki_base);
        Serial.print("│ Kd_base = ");
        Serial.println(Kd_base);
        Serial.print("│ ΔKp = ");
        Serial.println(dKp, 3);
        Serial.print("│ ΔKi = ");
        Serial.println(dKi, 3);
        Serial.print("│ ΔKd = ");
        Serial.println(dKd, 3);
        Serial.print("│ Kp_actual = ");
        Serial.println(Kp, 3);
        Serial.print("│ Ki_actual = ");
        Serial.println(Ki, 3);
        Serial.print("│ Kd_actual = ");
        Serial.println(Kd, 3);
        Serial.print("│ Setpoint = ");
        Serial.print(setpoint, 1);
        Serial.println("°");
        Serial.println("└────────────────────────────┘");
        Serial.println();
        break;
        
      case '+':
        setpoint += 1.0;
        Serial.print("📐 Setpoint: ");
        Serial.print(setpoint, 1);
        Serial.println("°");
        break;
        
      case '-':
        setpoint -= 1.0;
        Serial.print("📐 Setpoint: ");
        Serial.print(setpoint, 1);
        Serial.println("°");
        break;
        
      case '0':
        setpoint = 0.0;
        Serial.println("📐 Setpoint reset: 0.0°");
        break;
        
      case 'd':
      case 'D':
        debugMode = !debugMode;
        Serial.print("🔍 Debug mode: ");
        Serial.println(debugMode ? "ON" : "OFF");
        break;
        
      case 's':
      case 'S':
        balancing = false;
        motorsStop();
        integral = 0;
        Serial.println("🛑 STOP — Motor OFF, balancing OFF");
        break;
        
      default:
        Serial.print("❌ Unknown: '");
        Serial.print(cmd);
        Serial.println("' → ketik t/i/c/p/+/-/0/d/s");
        break;
    }
  }
}
