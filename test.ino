// test.ino - Motor driver test
// Hallo test from Lihana VPS

void setup() {
  Serial.begin(9600);
  Serial.println("hallo");
  Serial.println("Motor driver test ready...");
}

void loop() {
  // Loop test
  Serial.println("hallo dari loop");
  delay(2000);
}
