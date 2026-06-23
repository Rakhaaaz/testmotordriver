/*
 * ================================================================
 *  ESP32 + L298N Auto Motor Sequence Test
 * ================================================================
 *
 * Fungsi: Test otomatis semua kombinasi arah dan kecepatan motor.
 *         Berguna untuk verifikasi wiring dan arah roda sebelum
 *         menjalankan balancing controller.
 *
 * Sequence yang dijalankan:
 *   1. Roda Kiri MAJU  (2 detik)
 *   2. Roda Kiri MUNDUR (2 detik)
 *   3. Roda Kanan MAJU  (2 detik)
 *   4. Roda Kanan MUNDUR (2 detik)
 *   5. KEDUA MAJU (forward) (2 detik)
 *   6. KEDUA MUNDUR (backward) (2 detik)
 *   7. BELOK KIRI — kiri mundur, kanan maju (2 detik)
 *   8. BELOK KANAN — kiri maju, kanan mundur (2 detik)
 *   9. PWM SWEEP MAJU: 0→255→0 (ramp test)
 *  10. STOP
 *
 * Serial Commands:
 *   'r' = Ulangi sequence dari awal
 *   's' = Stop darurat
 *   'p' = Print status saat ini
 *
 * Wiring ESP32 → L298N:
 * ┌──────────┬──────────┐
 * │  ESP32   │  L298N   │
 * ├──────────┼──────────┤
 * │  GPIO 27 │  IN1     │  Motor A (Kiri)
 * │  GPIO 26 │  IN2     │  Motor A (Kiri)
 * │  GPIO 14 │  ENA     │  Motor A PWM
 * │  GPIO 25 │  IN3     │  Motor B (Kanan)
 * │  GPIO 33 │  IN4     │  Motor B (Kanan)
 * │  GPIO 32 │  ENB     │  Motor B PWM
 * │  GND     │  GND     │
 * └──────────┴──────────┘
 *
 * by Bara & Lihana
 */

#define IN1   27
#define IN2   26
#define ENA   14

#define IN3   25
#define IN4   33
#define ENB   32

#define PWM_FREQ       5000
#define PWM_RESOLUTION 8
#define TEST_SPEED     150
#define STEP_DELAY     2000
#define RAMP_STEP_MS   10

bool emergencyStop = false;
int currentStep = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  stopAll();

  Serial.println();
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║   ESP32 Auto Motor Sequence Test         ║");
  Serial.println("║   by Bara & Lihana                       ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.println();
  Serial.println("Commands: 'r'=Ulangi  's'=Stop  'p'=Status");
  Serial.println();
  Serial.println("Mulai sequence dalam 3 detik...");
  delay(3000);

  runSequence();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();

    switch (cmd) {
      case 'r':
      case 'R':
        emergencyStop = false;
        Serial.println("↩️  Ulangi sequence...");
        delay(1000);
        runSequence();
        break;

      case 's':
      case 'S':
        emergencyStop = true;
        stopAll();
        Serial.println("🛑 Emergency stop!");
        break;

      case 'p':
      case 'P':
        Serial.print("Step terakhir: ");
        Serial.println(currentStep);
        Serial.print("Emergency stop: ");
        Serial.println(emergencyStop ? "YES" : "NO");
        break;

      default:
        break;
    }
  }
}

void runSequence() {
  emergencyStop = false;

  printStep(1, "Roda KIRI MAJU");
  motorA(TEST_SPEED);
  motorB(0);
  waitOrAbort(STEP_DELAY);

  printStep(2, "Roda KIRI MUNDUR");
  motorA(-TEST_SPEED);
  motorB(0);
  waitOrAbort(STEP_DELAY);

  printStep(3, "Roda KANAN MAJU");
  motorA(0);
  motorB(TEST_SPEED);
  waitOrAbort(STEP_DELAY);

  printStep(4, "Roda KANAN MUNDUR");
  motorA(0);
  motorB(-TEST_SPEED);
  waitOrAbort(STEP_DELAY);

  printStep(5, "KEDUA MAJU (forward)");
  motorA(TEST_SPEED);
  motorB(TEST_SPEED);
  waitOrAbort(STEP_DELAY);

  printStep(6, "KEDUA MUNDUR (backward)");
  motorA(-TEST_SPEED);
  motorB(-TEST_SPEED);
  waitOrAbort(STEP_DELAY);

  printStep(7, "BELOK KIRI (kiri mundur, kanan maju)");
  motorA(-TEST_SPEED);
  motorB(TEST_SPEED);
  waitOrAbort(STEP_DELAY);

  printStep(8, "BELOK KANAN (kiri maju, kanan mundur)");
  motorA(TEST_SPEED);
  motorB(-TEST_SPEED);
  waitOrAbort(STEP_DELAY);

  printStep(9, "PWM SWEEP MAJU (ramp 0→255→0)");
  rampTest();

  stopAll();

  if (!emergencyStop) {
    Serial.println();
    Serial.println("✅ Sequence selesai! Ketik 'r' untuk ulangi.");
    Serial.println();
  }
}

void rampTest() {
  // Ramp up
  for (int pwm = 0; pwm <= 255; pwm += 5) {
    if (emergencyStop) return;
    motorA(pwm);
    motorB(pwm);
    Serial.print("  PWM: ");
    Serial.println(pwm);
    delay(RAMP_STEP_MS);
  }
  // Ramp down
  for (int pwm = 255; pwm >= 0; pwm -= 5) {
    if (emergencyStop) return;
    motorA(pwm);
    motorB(pwm);
    Serial.print("  PWM: ");
    Serial.println(pwm);
    delay(RAMP_STEP_MS);
  }
}

void motorA(int speed) {
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

void motorB(int speed) {
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

void stopAll() {
  motorA(0);
  motorB(0);
  Serial.println("  🛑 STOP");
}

void printStep(int step, const char* desc) {
  currentStep = step;
  Serial.print("─── Step ");
  Serial.print(step);
  Serial.print(": ");
  Serial.println(desc);
}

void waitOrAbort(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (Serial.available()) {
      char c = Serial.peek();
      if (c == 's' || c == 'S') {
        emergencyStop = true;
        stopAll();
        while (Serial.available()) Serial.read();
        return;
      }
    }
    delay(10);
  }
}