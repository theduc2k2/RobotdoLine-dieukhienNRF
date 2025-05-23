// Định nghĩa chân encoder (cho bánh phải)
#define ENCODER_A 2 // Encoder A (Interrupt 0)
#define ENCODER_B 3 // Encoder B (Interrupt 1)

// Biến đếm xung encoder
volatile long encoderCount = 0;

void setup() {
  Serial.begin(115200); // Baud rate cho Serial Monitor
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
  Serial.println("Bắt đầu đếm xung encoder. Quay bánh xe và theo dõi Serial Monitor.");
  Serial.println("Nhấn nút RESET trên board để đặt encoderCount về 0.");
}

void loop() {
  // In encoderCount mỗi 500ms
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= 500) {
    Serial.print("Encoder Count: ");
    Serial.println(encoderCount);
    lastPrintTime = currentTime;
  }
}

// Hàm ngắt xử lý xung encoder
void encoderISR() {
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderCount++; // Quay thuận
  } else {
    encoderCount--; // Quay ngược
  }
}