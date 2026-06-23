/*
 * ================================================================
 *  ESP32 + L298N Manual Motor Direction Test
 * ================================================================
 * 
 * Fungsi: Menguji arah putaran roda secara manual via Serial Monitor.
 * 
 * Command (ketik di Serial Monitor baud rate 115200):
 *   'a' = Roda Kiri MAJU
 *   'b' = Roda Kiri MUNDUR
 *   'c' = Roda Kanan MAJU
 *   'd' = Roda Kanan MUNDUR
 *   's' = STOP semua roda
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
 */

// Pin Roda Kiri (Motor A)
#define IN1   27
#define IN2   26
#define ENA   14

// Pin Roda Kanan (Motor B)
#define IN3   25
#define IN4   33
#define ENB   32

// PWM Config (ESP32 Core v3.x)
#define PWM_FREQ       5000
#define PWM_RESOLUTION 8
#define TEST_SPEED     150  // Kecepatan test (0-255)

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n==============================================");
  Serial.println("   ESP32 Manual Motor Tester Initialized");
  Serial.println("==============================================");
  Serial.println("Ketik perintah ini di Serial Monitor:");
  Serial.println("  'a' -> Roda Kiri MAJU");
  Serial.println("  'b' -> Roda Kiri MUNDUR");
  Serial.println("  'c' -> Roda Kanan MAJU");
  Serial.println("  'd' -> Roda Kanan MUNDUR");
  Serial.println("  's' -> STOP semua roda");
  Serial.println("==============================================");
  
  // Set pin motor sebagai OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Attach PWM ke pin Enable
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  
  // Pastikan motor mati saat awal
  stopAll();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // Hapus sisa karakter di buffer
    while (Serial.available()) Serial.read();
    
    switch (cmd) {
      case 'a':
      case 'A':
        Serial.println("◀️ [Kiri] MAJU");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(ENA, TEST_SPEED);
        break;
        
      case 'b':
      case 'B':
        Serial.println("◀️ [Kiri] MUNDUR");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        ledcWrite(ENA, TEST_SPEED);
        break;
        
      case 'c':
      case 'C':
        Serial.println("▶️ [Kanan] MAJU");
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        ledcWrite(ENB, TEST_SPEED);
        break;
        
      case 'd':
      case 'D':
        Serial.println("▶️ [Kanan] MUNDUR");
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(ENB, TEST_SPEED);
        break;
        
      case 's':
      case 'S':
        stopAll();
        break;
        
      default:
        Serial.print("❌ Perintah tidak dikenal: ");
        Serial.println(cmd);
        break;
    }
  }
}

void stopAll() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  Serial.println("🛑 [STOP ALL]");
}