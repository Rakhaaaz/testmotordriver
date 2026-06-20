#include <Wire.h>
#include <math.h>

#define IN1   27
#define IN2   26
#define ENA   14
#define IN3   25
#define IN4   33
#define ENB   32

#define SDA_PIN  21
#define SCL_PIN  22
#define MPU_ADDR 0x68

#define PWM_FREQ       5000
#define PWM_RESOLUTION 8

#define SAMPLE_TIME    5
#define PRINT_INTERVAL 200

float Kp_base = 25.0;
float Ki_base = 0.5;
float Kd_base = 15.0;

float Kp = 25.0;
float Ki = 0.5;
float Kd = 15.0;

float dKp = 0.0;
float dKi = 0.0;
float dKd = 0.0;

float setpoint = 0.0;
float error = 0.0;
float prev_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float pid_output = 0.0;

#define INTEGRAL_LIMIT 100.0
#define MOTOR_DEADZONE 30
#define MAX_TILT_ANGLE 45.0

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float gyroX_offset = 0.0;
float gyroY_offset = 0.0;
float gyroZ_offset = 0.0;
float accAngle = 0.0;
float gyroAngle = 0.0;
float currentAngle = 0.0;
float alpha = 0.98;

bool balancing = false;
bool debugMode = false;
unsigned long lastTime = 0;
unsigned long lastPrint = 0;
float dt = 0.005;

#define F_NB  -3.0
#define F_NM  -2.0
#define F_NS  -1.0
#define F_ZE   0.0
#define F_PS   1.0
#define F_PM   2.0
#define F_PB   3.0

#define E_MAX   30.0
#define DE_MAX  50.0

#define SCALE_KP  5.0
#define SCALE_KI  0.2
#define SCALE_KD  3.0

float trimf(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0;
  if (x <= b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

float left_mf(float x, float a, float b) {
  if (x <= a) return 1.0;
  if (x >= b) return 0.0;
  return (b - x) / (b - a);
}

float right_mf(float x, float a, float b) {
  if (x <= a) return 0.0;
  if (x >= b) return 1.0;
  return (x - a) / (b - a);
}

void fuzzify(float input, float max_val, float membership[7]) {
  float x = constrain(input / max_val, -1.0, 1.0);
  membership[0] = left_mf(x, -1.0, -0.67);
  membership[1] = trimf(x, -1.0, -0.67, -0.33);
  membership[2] = trimf(x, -0.67, -0.33, 0.0);
  membership[3] = trimf(x, -0.33, 0.0, 0.33);
  membership[4] = trimf(x, 0.0, 0.33, 0.67);
  membership[5] = trimf(x, 0.33, 0.67, 1.0);
  membership[6] = right_mf(x, 0.67, 1.0);
}

const float rules_dKp[7][7] = {
  {F_PB, F_PB, F_PM, F_PM, F_PS, F_ZE, F_ZE},
  {F_PB, F_PB, F_PM, F_PS, F_PS, F_ZE, F_NS},
  {F_PM, F_PM, F_PM, F_PS, F_ZE, F_NS, F_NS},
  {F_PM, F_PM, F_PS, F_ZE, F_NS, F_NM, F_NM},
  {F_PS, F_PS, F_ZE, F_NS, F_NS, F_NM, F_NM},
  {F_PS, F_ZE, F_NS, F_NM, F_NM, F_NM, F_NB},
  {F_ZE, F_ZE, F_NM, F_NM, F_NM, F_NB, F_NB}
};

const float rules_dKi[7][7] = {
  {F_NB, F_NB, F_NM, F_NM, F_NS, F_ZE, F_ZE},
  {F_NB, F_NB, F_NM, F_NS, F_NS, F_ZE, F_ZE},
  {F_NB, F_NM, F_NS, F_NS, F_ZE, F_PS, F_PS},
  {F_NM, F_NM, F_NS, F_ZE, F_PS, F_PM, F_PM},
  {F_NM, F_NS, F_ZE, F_PS, F_PS, F_PM, F_PB},
  {F_ZE, F_ZE, F_PS, F_PS, F_PM, F_PB, F_PB},
  {F_ZE, F_ZE, F_PS, F_PM, F_PM, F_PB, F_PB}
};

const float rules_dKd[7][7] = {
  {F_PS, F_NS, F_NB, F_NB, F_NB, F_NM, F_PS},
  {F_PS, F_NS, F_NB, F_NM, F_NM, F_NS, F_ZE},
  {F_ZE, F_NS, F_NM, F_NM, F_NS, F_NS, F_ZE},
  {F_ZE, F_NS, F_NS, F_NS, F_NS, F_NS, F_ZE},
  {F_ZE, F_ZE, F_ZE, F_ZE, F_ZE, F_ZE, F_ZE},
  {F_NB, F_NS, F_NS, F_NS, F_ZE, F_PS, F_PB},
  {F_NB, F_NM, F_NM, F_NS, F_ZE, F_PS, F_PB}
};

float fuzzyInference(float e_membership[7], float de_membership[7], const float rules[7][7]) {
  float numerator = 0.0;
  float denominator = 0.0;
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 7; j++) {
      float firing_strength = min(e_membership[i], de_membership[j]);
      if (firing_strength > 0.001) {
        numerator += firing_strength * rules[i][j];
        denominator += firing_strength;
      }
    }
  }
  if (denominator < 0.001) return 0.0;
  return numerator / denominator;
}

void fuzzyPIDCompute(float e, float de) {
  float e_mf[7], de_mf[7];
  fuzzify(e, E_MAX, e_mf);
  fuzzify(de, DE_MAX, de_mf);

  dKp = fuzzyInference(e_mf, de_mf, rules_dKp) * SCALE_KP;
  dKi = fuzzyInference(e_mf, de_mf, rules_dKi) * SCALE_KI;
  dKd = fuzzyInference(e_mf, de_mf, rules_dKd) * SCALE_KD;

  Kp = max(0.0f, Kp_base + dKp);
  Ki = max(0.0f, Ki_base + dKi);
  Kd = max(0.0f, Kd_base + dKd);
}

void mpuInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);
}

void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t rawAccX  = Wire.read() << 8 | Wire.read();
  int16_t rawAccY  = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ  = Wire.read() << 8 | Wire.read();
  int16_t rawTemp  = Wire.read() << 8 | Wire.read();
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

  accX = rawAccX / 16384.0;
  accY = rawAccY / 16384.0;
  accZ = rawAccZ / 16384.0;
  gyroX = (rawGyroX / 131.0) - gyroX_offset;
  gyroY = (rawGyroY / 131.0) - gyroY_offset;
  gyroZ = (rawGyroZ / 131.0) - gyroZ_offset;
}

void mpuCalibrate() {
  Serial.println("Calibrating...");
  float sumGX = 0, sumGY = 0, sumGZ = 0;
  int samples = 500;

  for (int i = 0; i < samples; i++) {
    mpuRead();
    sumGX += (gyroX + gyroX_offset);
    sumGY += (gyroY + gyroY_offset);
    sumGZ += (gyroZ + gyroZ_offset);
    delay(2);
  }

  gyroX_offset = sumGX / samples;
  gyroY_offset = sumGY / samples;
  gyroZ_offset = sumGZ / samples;

  mpuRead();
  currentAngle = atan2(accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  gyroAngle = currentAngle;

  Serial.print("Offset: ");
  Serial.print(gyroX_offset, 2);
  Serial.print(", ");
  Serial.print(gyroY_offset, 2);
  Serial.print(", ");
  Serial.println(gyroZ_offset, 2);
  Serial.print("Initial angle: ");
  Serial.print(currentAngle, 1);
  Serial.println(" deg");
}

float getAngle() {
  mpuRead();
  accAngle = atan2(accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  gyroAngle += gyroY * dt;
  currentAngle = alpha * (currentAngle + gyroY * dt) + (1.0 - alpha) * accAngle;
  return currentAngle;
}

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
  if (pwm > 0) pwm += MOTOR_DEADZONE;
  else if (pwm < 0) pwm -= MOTOR_DEADZONE;
  pwm = constrain(pwm, -255, 255);
  motorA_drive(pwm);
  motorB_drive(pwm);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Self-Balancing Robot - Fuzzy-PID");
  Serial.println();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  motorsStop();

  mpuInit();
  mpuCalibrate();

  Serial.print("Kp=");
  Serial.print(Kp_base);
  Serial.print(" Ki=");
  Serial.print(Ki_base);
  Serial.print(" Kd=");
  Serial.println(Kd_base);
  Serial.print("Setpoint=");
  Serial.print(setpoint);
  Serial.println(" deg");
  Serial.println();
  Serial.println("Commands: t=toggle, i=info, c=calibrate, p=params");
  Serial.println("          +/-/0=setpoint, d=debug, s=stop");
  Serial.println();

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastTime >= SAMPLE_TIME) {
    dt = (now - lastTime) / 1000.0;
    lastTime = now;

    float angle = getAngle();

    if (balancing) {
      if (abs(angle - setpoint) > MAX_TILT_ANGLE) {
        motorsStop();
        balancing = false;
        Serial.println("TILT > 45 deg - Motor OFF");
        integral = 0;
        return;
      }

      error = angle - setpoint;
      derivative = (error - prev_error) / dt;

      fuzzyPIDCompute(error, derivative);

      integral += error * dt;
      integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

      pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
      pid_output = constrain(pid_output, -255.0, 255.0);

      motorsDrive(pid_output);
      prev_error = error;

      if (debugMode && (now - lastPrint >= PRINT_INTERVAL)) {
        lastPrint = now;
        Serial.print("A:");
        Serial.print(angle, 1);
        Serial.print(" E:");
        Serial.print(error, 1);
        Serial.print(" Kp:");
        Serial.print(Kp, 1);
        Serial.print(" Ki:");
        Serial.print(Ki, 2);
        Serial.print(" Kd:");
        Serial.print(Kd, 1);
        Serial.print(" Out:");
        Serial.println(pid_output, 0);
      }
    }
  }

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
          Serial.println("Balancing ON");
        } else {
          motorsStop();
          Serial.println("Balancing OFF");
        }
        break;

      case 'i':
      case 'I':
        Serial.print("Angle=");
        Serial.print(currentAngle, 2);
        Serial.print(" Setpoint=");
        Serial.print(setpoint, 1);
        Serial.print(" Error=");
        Serial.print(error, 2);
        Serial.print(" PID=");
        Serial.print(pid_output, 1);
        Serial.print(" Bal=");
        Serial.println(balancing ? "ON" : "OFF");
        Serial.print("Kp=");
        Serial.print(Kp, 2);
        Serial.print("(d");
        Serial.print(dKp, 2);
        Serial.print(") Ki=");
        Serial.print(Ki, 3);
        Serial.print("(d");
        Serial.print(dKi, 3);
        Serial.print(") Kd=");
        Serial.print(Kd, 2);
        Serial.print("(d");
        Serial.print(dKd, 2);
        Serial.println(")");
        break;

      case 'c':
      case 'C':
        balancing = false;
        motorsStop();
        Serial.println("Re-calibrating...");
        delay(2000);
        mpuCalibrate();
        integral = 0;
        prev_error = 0;
        break;

      case 'p':
      case 'P':
        Serial.print("Base: Kp=");
        Serial.print(Kp_base);
        Serial.print(" Ki=");
        Serial.print(Ki_base);
        Serial.print(" Kd=");
        Serial.println(Kd_base);
        Serial.print("Delta: dKp=");
        Serial.print(dKp, 3);
        Serial.print(" dKi=");
        Serial.print(dKi, 3);
        Serial.print(" dKd=");
        Serial.println(dKd, 3);
        Serial.print("Actual: Kp=");
        Serial.print(Kp, 3);
        Serial.print(" Ki=");
        Serial.print(Ki, 3);
        Serial.print(" Kd=");
        Serial.println(Kd, 3);
        break;

      case '+':
        setpoint += 1.0;
        Serial.print("Setpoint: ");
        Serial.println(setpoint, 1);
        break;

      case '-':
        setpoint -= 1.0;
        Serial.print("Setpoint: ");
        Serial.println(setpoint, 1);
        break;

      case '0':
        setpoint = 0.0;
        Serial.println("Setpoint: 0.0");
        break;

      case 'd':
      case 'D':
        debugMode = !debugMode;
        Serial.print("Debug: ");
        Serial.println(debugMode ? "ON" : "OFF");
        break;

      case 's':
      case 'S':
        balancing = false;
        motorsStop();
        integral = 0;
        Serial.println("STOP");
        break;
    }
  }
}
