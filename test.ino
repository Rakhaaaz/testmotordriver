/*
 * ============================================
 *  ESP32 + L298N Motor Driver - Output Tester
 * ============================================
 * 
 * Fungsi: Test semua output L298N (Motor A & B)
 *         - Forward, Reverse, Stop
 *         - PWM speed control
 *         - Enable pin test
 * 
 * Wiring ESP32 → L298N:
 * ┌──────────┬──────────┐
 * │  ESP32   │  L298N   │
 * ├──────────┼──────────┤
 * │  GPIO 27 │  IN1     │  Motor A direction
 * │  GPIO 26 │  IN2     │  Motor A direction
 * │  GPIO 14 │  ENA     │  Motor A speed (PWM)
 * │  GPIO 25 │  IN3     │  Motor B direction
 * │  GPIO 33 │  IN4     │  Motor B direction
 * │  GPIO 32 │  ENB     │  Motor B speed (PWM)
 * │  GND     │  GND     │  Common ground
 * └──────────┴──────────┘
 * 
 * Power:
 *   - L298N VCC  → 7-12V (battery/adaptor)
 *   - L298N 5V   → bisa supply ke ESP32 (optional)
 *   - L298N GND  → ESP32 GND (WAJIB sama)
 * 
 * Buka Serial Monitor: 115200 baud
 * Ketik command untuk test manual:
 *   'a' = Test Motor A
 *   'b' = Test Motor B  
 *   'c' = Test Both Motors
 *   'p' = PWM Speed Test (gradual)
 *   's' = Stop All
 *   'f' = Full Auto Test (semua)
 */

// ========== PIN DEFINITIONS ==========
// Motor A
#define IN1   27    // Direction pin 1
#define IN2   26    // Direction pin 2
#define ENA   14    // Enable / PWM speed

// Motor B
#define IN3   25    // Direction pin 1
#define IN4   33    // Direction pin 2
#define ENB   32    // Enable / PWM speed

// ========== PWM CONFIG (ESP32 LEDC) ==========
#define PWM_FREQ      5000    // 5 KHz
#define PWM_RESOLUTION 8      // 8-bit (0-255)
#define PWM_CHANNEL_A  0      // LEDC channel for ENA
#define PWM_CHANNEL_B  1      // LEDC channel for ENB

// ========== TEST TIMING ==========
#define TEST_DELAY    2000    // Durasi tiap test step (ms)
#define RAMP_DELAY    30      // Delay antar step PWM ramp (ms)

// ========== FUNCTION DECLARATIONS ==========
void motorA_forward(int speed);
void motorA_reverse(int speed);
void motorA_stop();
void motorB_forward(int speed);
void motorB_reverse(int speed);
void motorB_stop();
void stopAll();
void testMotorA();
void testMotorB();
void testBothMotors();
void testPWMRamp();
void fullAutoTest();
void printHeader(const char* title);
void printPinStatus();

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Header
  Serial.println();
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║  ESP32 + L298N Motor Driver Tester       ║");
  Serial.println("║  by Bara & Lihana                        ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.println();
  
  // Setup direction pins as OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Setup PWM channels (ESP32 LEDC)
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcAttachPin(ENB, PWM_CHANNEL_B);
  
  // Pastikan semua OFF dulu
  stopAll();
  
  // Pin status check
  printPinStatus();
  
  // Menu
  Serial.println("┌─────────── MENU ───────────┐");
  Serial.println("│ 'a' = Test Motor A          │");
  Serial.println("│ 'b' = Test Motor B          │");
  Serial.println("│ 'c' = Test Both Motors      │");
  Serial.println("│ 'p' = PWM Speed Ramp Test   │");
  Serial.println("│ 's' = Stop All              │");
  Serial.println("│ 'f' = Full Auto Test        │");
  Serial.println("└─────────────────────────────┘");
  Serial.println();
  Serial.println(">> Ketik command di Serial Monitor...");
  Serial.println(">> Atau tunggu 5 detik untuk auto test...");
  Serial.println();
  
  // Auto test setelah 5 detik kalo gak ada input
  unsigned long startWait = millis();
  while (millis() - startWait < 5000) {
    if (Serial.available()) {
      return;  // Ada input, skip auto test
    }
    delay(100);
  }
  
  // Gak ada input, jalanin full auto test
  Serial.println("⏱️  Timeout - menjalankan Full Auto Test...");
  Serial.println();
  fullAutoTest();
}

// ========== LOOP ==========
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // Flush buffer
    while (Serial.available()) Serial.read();
    
    switch (cmd) {
      case 'a':
      case 'A':
        testMotorA();
        break;
        
      case 'b':
      case 'B':
        testMotorB();
        break;
        
      case 'c':
      case 'C':
        testBothMotors();
        break;
        
      case 'p':
      case 'P':
        testPWMRamp();
        break;
        
      case 's':
      case 'S':
        stopAll();
        Serial.println("🛑 All motors STOPPED");
        break;
        
      case 'f':
      case 'F':
        fullAutoTest();
        break;
        
      default:
        Serial.print("❌ Command tidak dikenal: '");
        Serial.print(cmd);
        Serial.println("'");
        Serial.println("   Ketik: a, b, c, p, s, atau f");
        break;
    }
    
    Serial.println();
    Serial.println(">> Siap menerima command berikutnya...");
    Serial.println();
  }
}

// ========== MOTOR CONTROL FUNCTIONS ==========

void motorA_forward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, speed);
}

void motorA_reverse(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM_CHANNEL_A, speed);
}

void motorA_stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
}

void motorB_forward(int speed) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void motorB_reverse(int speed) {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void motorB_stop() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_B, 0);
}

void stopAll() {
  motorA_stop();
  motorB_stop();
}

// ========== TEST FUNCTIONS ==========

void testMotorA() {
  printHeader("TEST MOTOR A");
  
  Serial.println("  [1/4] Motor A → FORWARD (speed 200)");
  motorA_forward(200);
  delay(TEST_DELAY);
  
  Serial.println("  [2/4] Motor A → STOP");
  motorA_stop();
  delay(1000);
  
  Serial.println("  [3/4] Motor A → REVERSE (speed 200)");
  motorA_reverse(200);
  delay(TEST_DELAY);
  
  Serial.println("  [4/4] Motor A → STOP");
  motorA_stop();
  delay(500);
  
  Serial.println("  ✅ Motor A test SELESAI");
  Serial.println();
}

void testMotorB() {
  printHeader("TEST MOTOR B");
  
  Serial.println("  [1/4] Motor B → FORWARD (speed 200)");
  motorB_forward(200);
  delay(TEST_DELAY);
  
  Serial.println("  [2/4] Motor B → STOP");
  motorB_stop();
  delay(1000);
  
  Serial.println("  [3/4] Motor B → REVERSE (speed 200)");
  motorB_reverse(200);
  delay(TEST_DELAY);
  
  Serial.println("  [4/4] Motor B → STOP");
  motorB_stop();
  delay(500);
  
  Serial.println("  ✅ Motor B test SELESAI");
  Serial.println();
}

void testBothMotors() {
  printHeader("TEST BOTH MOTORS");
  
  Serial.println("  [1/6] Both → FORWARD (speed 200)");
  motorA_forward(200);
  motorB_forward(200);
  delay(TEST_DELAY);
  
  Serial.println("  [2/6] Both → STOP");
  stopAll();
  delay(1000);
  
  Serial.println("  [3/6] Both → REVERSE (speed 200)");
  motorA_reverse(200);
  motorB_reverse(200);
  delay(TEST_DELAY);
  
  Serial.println("  [4/6] Both → STOP");
  stopAll();
  delay(1000);
  
  Serial.println("  [5/6] A=FORWARD, B=REVERSE (belok kiri)");
  motorA_forward(200);
  motorB_reverse(200);
  delay(TEST_DELAY);
  
  Serial.println("  [6/6] A=REVERSE, B=FORWARD (belok kanan)");
  motorA_reverse(200);
  motorB_forward(200);
  delay(TEST_DELAY);
  
  stopAll();
  Serial.println("  ✅ Both motors test SELESAI");
  Serial.println();
}

void testPWMRamp() {
  printHeader("PWM SPEED RAMP TEST");
  
  // Motor A ramp up
  Serial.println("  Motor A → Ramp UP (0 → 255)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  for (int speed = 0; speed <= 255; speed += 5) {
    ledcWrite(PWM_CHANNEL_A, speed);
    Serial.print("    Speed: ");
    Serial.print(speed);
    Serial.print("/255 (");
    Serial.print((speed * 100) / 255);
    Serial.println("%)");
    delay(RAMP_DELAY);
  }
  
  // Motor A ramp down
  Serial.println("  Motor A → Ramp DOWN (255 → 0)");
  for (int speed = 255; speed >= 0; speed -= 5) {
    ledcWrite(PWM_CHANNEL_A, speed);
    delay(RAMP_DELAY);
  }
  motorA_stop();
  delay(500);
  
  // Motor B ramp up
  Serial.println("  Motor B → Ramp UP (0 → 255)");
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  for (int speed = 0; speed <= 255; speed += 5) {
    ledcWrite(PWM_CHANNEL_B, speed);
    Serial.print("    Speed: ");
    Serial.print(speed);
    Serial.print("/255 (");
    Serial.print((speed * 100) / 255);
    Serial.println("%)");
    delay(RAMP_DELAY);
  }
  
  // Motor B ramp down
  Serial.println("  Motor B → Ramp DOWN (255 → 0)");
  for (int speed = 255; speed >= 0; speed -= 5) {
    ledcWrite(PWM_CHANNEL_B, speed);
    delay(RAMP_DELAY);
  }
  motorB_stop();
  
  Serial.println("  ✅ PWM ramp test SELESAI");
  Serial.println();
}

void fullAutoTest() {
  printHeader("FULL AUTO TEST - SEMUA OUTPUT");
  
  Serial.println("━━━━━━ TAHAP 1: Individual Pin Test ━━━━━━");
  Serial.println();
  
  // Test setiap pin satu-satu
  Serial.println("  [PIN] IN1 (GPIO 27) → HIGH");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  delay(1000);
  Serial.println("    → Cek: IN1 harusnya HIGH (3.3V)");
  
  Serial.println("  [PIN] IN2 (GPIO 26) → HIGH");
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  delay(1000);
  Serial.println("    → Cek: IN2 harusnya HIGH (3.3V)");
  
  Serial.println("  [PIN] IN3 (GPIO 25) → HIGH");
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(1000);
  Serial.println("    → Cek: IN3 harusnya HIGH (3.3V)");
  
  Serial.println("  [PIN] IN4 (GPIO 33) → HIGH");
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  delay(1000);
  Serial.println("    → Cek: IN4 harusnya HIGH (3.3V)");
  
  Serial.println("  [PIN] ENA (GPIO 14) → PWM 255");
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, 255);
  delay(1000);
  Serial.println("    → Cek: ENA harusnya HIGH (3.3V)");
  
  Serial.println("  [PIN] ENB (GPIO 32) → PWM 255");
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 255);
  delay(1000);
  Serial.println("    → Cek: ENB harusnya HIGH (3.3V)");
  
  ledcWrite(PWM_CHANNEL_B, 0);
  stopAll();
  Serial.println("  ✅ Individual pin test DONE");
  Serial.println();
  
  // ---- TAHAP 2 ----
  Serial.println("━━━━━━ TAHAP 2: Motor A Test ━━━━━━");
  testMotorA();
  
  // ---- TAHAP 3 ----
  Serial.println("━━━━━━ TAHAP 3: Motor B Test ━━━━━━");
  testMotorB();
  
  // ---- TAHAP 4 ----
  Serial.println("━━━━━━ TAHAP 4: Both Motors Test ━━━━━━");
  testBothMotors();
  
  // ---- TAHAP 5 ----
  Serial.println("━━━━━━ TAHAP 5: PWM Speed Test ━━━━━━");
  testPWMRamp();
  
  // ---- TAHAP 6: BRAKE TEST ----
  Serial.println("━━━━━━ TAHAP 6: Brake Test ━━━━━━");
  Serial.println();
  
  Serial.println("  [BRAKE] Motor A → Full speed lalu brake");
  motorA_forward(255);
  delay(1500);
  // Brake = kedua IN HIGH
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM_CHANNEL_A, 255);
  Serial.println("    → IN1=HIGH, IN2=HIGH = BRAKE (motor ngerem)");
  delay(1500);
  motorA_stop();
  delay(500);
  
  Serial.println("  [BRAKE] Motor B → Full speed lalu brake");
  motorB_forward(255);
  delay(1500);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  ledcWrite(PWM_CHANNEL_B, 255);
  Serial.println("    → IN3=HIGH, IN4=HIGH = BRAKE (motor ngerem)");
  delay(1500);
  motorB_stop();
  
  Serial.println("  ✅ Brake test DONE");
  Serial.println();
  
  // ---- SUMMARY ----
  stopAll();
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║         FULL AUTO TEST SELESAI!          ║");
  Serial.println("╠══════════════════════════════════════════╣");
  Serial.println("║  Checklist:                              ║");
  Serial.println("║  □ Motor A berputar forward?             ║");
  Serial.println("║  □ Motor A berputar reverse?             ║");
  Serial.println("║  □ Motor B berputar forward?             ║");
  Serial.println("║  □ Motor B berputar reverse?             ║");
  Serial.println("║  □ PWM speed bisa diatur?                ║");
  Serial.println("║  □ Brake berfungsi (motor ngerem)?       ║");
  Serial.println("║  □ Kedua motor bisa jalan bareng?        ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.println();
}

// ========== UTILITY FUNCTIONS ==========

void printHeader(const char* title) {
  Serial.println();
  Serial.print("┌─── ");
  Serial.print(title);
  Serial.println(" ───┐");
  Serial.println();
}

void printPinStatus() {
  Serial.println("┌─────── PIN CONFIG ────────┐");
  Serial.print("│ Motor A: IN1=GPIO");
  Serial.print(IN1);
  Serial.print(" IN2=GPIO");
  Serial.print(IN2);
  Serial.print(" ENA=GPIO");
  Serial.println(IN1 == 27 ? "14 │" : "?  │");
  Serial.print("│ Motor B: IN3=GPIO");
  Serial.print(IN3);
  Serial.print(" IN4=GPIO");
  Serial.print(IN4);
  Serial.print(" ENB=GPIO");
  Serial.println(ENB == 32 ? "32 │" : "?  │");
  Serial.print("│ PWM Freq: ");
  Serial.print(PWM_FREQ);
  Serial.println("Hz              │");
  Serial.print("│ PWM Resolution: ");
  Serial.print(PWM_RESOLUTION);
  Serial.println("-bit (0-255)  │");
  Serial.println("└────────────────────────────┘");
  Serial.println();
}
