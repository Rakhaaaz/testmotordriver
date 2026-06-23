#define IN1   27
#define IN2   26
#define ENA   14
#define IN3   25
#define IN4   33
#define ENB   32

#define PWM_FREQ       5000
#define PWM_RESOLUTION 8
#define TEST_DELAY     2000
#define RAMP_DELAY     30

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

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ESP32 + L298N Motor Driver Test");
  Serial.println();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  stopAll();

  Serial.println("Commands:");
  Serial.println("  a = Test Motor A");
  Serial.println("  b = Test Motor B");
  Serial.println("  c = Test Both Motors");
  Serial.println("  p = PWM Speed Ramp");
  Serial.println("  s = Stop All");
  Serial.println("  f = Full Auto Test");
  Serial.println();

  unsigned long startWait = millis();
  while (millis() - startWait < 5000) {
    if (Serial.available()) return;
    delay(100);
  }

  Serial.println("Auto test starting...");
  fullAutoTest();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();

    switch (cmd) {
      case 'a': case 'A': testMotorA(); break;
      case 'b': case 'B': testMotorB(); break;
      case 'c': case 'C': testBothMotors(); break;
      case 'p': case 'P': testPWMRamp(); break;
      case 's': case 'S':
        stopAll();
        Serial.println("All motors stopped");
        break;
      case 'f': case 'F': fullAutoTest(); break;
    }
  }
}

void motorA_forward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, speed);
}

void motorA_reverse(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(ENA, speed);
}

void motorA_stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, 0);
}

void motorB_forward(int speed) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENB, speed);
}

void motorB_reverse(int speed) {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENB, speed);
}

void motorB_stop() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENB, 0);
}

void stopAll() {
  motorA_stop();
  motorB_stop();
}

void testMotorA() {
  Serial.println("Motor A - Forward");
  motorA_forward(200);
  delay(TEST_DELAY);

  Serial.println("Motor A - Stop");
  motorA_stop();
  delay(1000);

  Serial.println("Motor A - Reverse");
  motorA_reverse(200);
  delay(TEST_DELAY);

  Serial.println("Motor A - Stop");
  motorA_stop();
  delay(500);
  Serial.println("Motor A test done");
}

void testMotorB() {
  Serial.println("Motor B - Forward");
  motorB_forward(200);
  delay(TEST_DELAY);

  Serial.println("Motor B - Stop");
  motorB_stop();
  delay(1000);

  Serial.println("Motor B - Reverse");
  motorB_reverse(200);
  delay(TEST_DELAY);

  Serial.println("Motor B - Stop");
  motorB_stop();
  delay(500);
  Serial.println("Motor B test done");
}

void testBothMotors() {
  Serial.println("Both - Forward");
  motorA_forward(200);
  motorB_forward(200);
  delay(TEST_DELAY);

  Serial.println("Both - Stop");
  stopAll();
  delay(1000);

  Serial.println("Both - Reverse");
  motorA_reverse(200);
  motorB_reverse(200);
  delay(TEST_DELAY);

  Serial.println("Both - Stop");
  stopAll();
  delay(1000);

  Serial.println("Turn Left (A fwd, B rev)");
  motorA_forward(200);
  motorB_reverse(200);
  delay(TEST_DELAY);

  Serial.println("Turn Right (A rev, B fwd)");
  motorA_reverse(200);
  motorB_forward(200);
  delay(TEST_DELAY);

  stopAll();
  Serial.println("Both motors test done");
}

void testPWMRamp() {
  Serial.println("Motor A - Ramp UP");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  for (int speed = 0; speed <= 255; speed += 5) {
    ledcWrite(ENA, speed);
    delay(RAMP_DELAY);
  }

  Serial.println("Motor A - Ramp DOWN");
  for (int speed = 255; speed >= 0; speed -= 5) {
    ledcWrite(ENA, speed);
    delay(RAMP_DELAY);
  }
  motorA_stop();
  delay(500);

  Serial.println("Motor B - Ramp UP");
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  for (int speed = 0; speed <= 255; speed += 5) {
    ledcWrite(ENB, speed);
    delay(RAMP_DELAY);
  }

  Serial.println("Motor B - Ramp DOWN");
  for (int speed = 255; speed >= 0; speed -= 5) {
    ledcWrite(ENB, speed);
    delay(RAMP_DELAY);
  }
  motorB_stop();
  Serial.println("PWM ramp test done");
}

void fullAutoTest() {
  Serial.println("=== Pin Test ===");

  Serial.println("IN1 HIGH");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0); ledcWrite(ENB, 0);
  delay(1000);

  Serial.println("IN2 HIGH");
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  delay(1000);

  Serial.println("IN3 HIGH");
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(1000);

  Serial.println("IN4 HIGH");
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  delay(1000);

  Serial.println("ENA PWM 255");
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 255);
  delay(1000);

  Serial.println("ENB PWM 255");
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 255);
  delay(1000);

  ledcWrite(ENB, 0);
  stopAll();
  Serial.println("Pin test done");
  Serial.println();

  Serial.println("=== Motor A ===");
  testMotorA();
  Serial.println();

  Serial.println("=== Motor B ===");
  testMotorB();
  Serial.println();

  Serial.println("=== Both Motors ===");
  testBothMotors();
  Serial.println();

  Serial.println("=== PWM Ramp ===");
  testPWMRamp();
  Serial.println();

  Serial.println("=== Brake Test ===");
  Serial.println("Motor A full speed then brake");
  motorA_forward(255);
  delay(1500);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  ledcWrite(ENA, 255);
  delay(1500);
  motorA_stop();
  delay(500);

  Serial.println("Motor B full speed then brake");
  motorB_forward(255);
  delay(1500);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENB, 255);
  delay(1500);
  motorB_stop();

  stopAll();
  Serial.println();
  Serial.println("All tests complete");
}
