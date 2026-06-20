// test.ino - Motor driver test
// Hallo test from Lihana VPS

const int MOTOR_PIN = 9;
const int DIR_PIN   = 8;

void setup() {
  Serial.begin(9600);
  Serial.println("hallo");
  Serial.println("Motor driver test ready...");
  
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
}

void loop() {
  // Test hallo di loop
  Serial.println("hallo dari loop");
  delay(2000);
  
  // Test motor forward
  Serial.println("Motor FORWARD");
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(MOTOR_PIN, 150);
  delay(2000);
  
  // Stop
  Serial.println("Motor STOP");
  analogWrite(MOTOR_PIN, 0);
  delay(1000);
  
  // Test motor reverse
  Serial.println("Motor REVERSE");
  digitalWrite(DIR_PIN, LOW);
  analogWrite(MOTOR_PIN, 150);
  delay(2000);
  
  // Stop
  analogWrite(MOTOR_PIN, 0);
  delay(2000);
}
