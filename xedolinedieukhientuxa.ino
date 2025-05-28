#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(17, 10); // CE: A3 (17), CSN: D10 (10)

const byte address[6] = "00001"; // Địa chỉ kênh giao tiếp
bool isConnected = false; // Trạng thái kết nối
int mode = -1; // -1: Chưa chọn chế độ, 0: Điều khiển tay, 1: Dò line
unsigned long lastLogTime = 0; // Thời gian log lần cuối
const unsigned long logInterval = 100; // Log mỗi 100ms
unsigned long lastAvoidTime = 0;

// Định nghĩa chân cho HC-SR04
#define TRIG_PIN A5 // Chân Trig của HC-SR04
#define ECHO_PIN A4 // Chân Echo của HC-SR04
const float OBSTACLE_DISTANCE = 6.0; // Khoảng cách né tránh vật cản (cm)

// Định nghĩa chân encoder
#define ENCODER_A 2 // Encoder A (Interrupt 0)
#define ENCODER_B 3 // Encoder B (Interrupt 1)
volatile long encoderCount = 0; // Biến đếm xung encoder
const int PULSES_PER_REV = 1440; // Số xung mỗi vòng bánh xe
const int PULSES_FOR_90_DEG = 100; // Số xung để quay 90 độ
const int PULSES_FOR_10CM = 5000; // Số xung để đi 10cm
const unsigned long DEBOUNCE_DELAY = 100; // Microseconds, chống rung
bool isAvoidingObstacle = false;
float lastLinePosition = 0;
// Định nghĩa chân LED
#define LED_PIN 4 // Chân D4 cho LED báo chuyển mode

struct DataPacket {
  int joystickX;
  int joystickY;
  bool buttonA;
  bool buttonB;
  bool buttonC;
  bool buttonD;
  bool buttonE;
  bool buttonF;
  bool ping; // Tín hiệu ping để kiểm tra kết nối
};
DataPacket data;

#define IN1 6 // Motor B (phải, OUT1)
#define IN2 7 // Motor B (phải, OUT2)
#define IN3 8 // Motor A (trái, OUT3)
#define IN4 9 // Motor A (trái, OUT4)

#define LINE_OUT1 A0 // Cảm biến dò line (cực trái)
#define LINE_OUT2 A1
#define LINE_OUT3 A2 // Giữa
#define LINE_OUT4 A6
#define LINE_OUT5 A7 // Cực phải

float baseSpeed = 80; // Tốc độ cơ bản
const int minSpeed = -120;
const int maxSpeed = 120;

// Hàm đọc cảm biến line và tính tổng
void readLineSensors(int &s1, int &s2, int &s3, int &s4, int &s5, int &sum) {
  s1 = analogRead(LINE_OUT1) > 500 ? 1 : 0;
  s2 = analogRead(LINE_OUT2) > 500 ? 1 : 0;
  s3 = analogRead(LINE_OUT3) > 500 ? 1 : 0;
  s4 = analogRead(LINE_OUT4) > 500 ? 1 : 0;
  s5 = analogRead(LINE_OUT5) > 500 ? 1 : 0;
  sum = s1 + s2 + s3 + s4 + s5;
}

// ĐỘNG CƠ
void motorSetup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();
}

void moveForward() {
  Motor_left(baseSpeed);
  Motor_right(baseSpeed);
}

void moveBackward() {
  Motor_left(-baseSpeed);
  Motor_right(-baseSpeed);
}

void turnLeft() {
  Motor_left(-baseSpeed);
  Motor_right(baseSpeed);
}

void turnRight() {
  Motor_left(baseSpeed);
  Motor_right(-baseSpeed);
}

void Motor_left(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}

void Motor_right(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

void stopMotors() {
  Motor_left(0);
  Motor_right(0);
}

// ENCODER
void encoderSetup() {
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
}

void encoderISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime < DEBOUNCE_DELAY) return;
  if (digitalRead(ENCODER_B) == HIGH) encoderCount++;
  else encoderCount--;
  lastInterruptTime = currentTime;
}

// HÀM QUAY 90 ĐỘ (XOAY PHẢI)
bool turn90Degrees() {
  encoderCount = 0;
  unsigned long startTime = millis();
  int targetPulses = PULSES_FOR_90_DEG;
  const unsigned long timeout = 900;
  int motorSpeed = baseSpeed;

  Motor_left(motorSpeed);
  Motor_right(0);
  Serial.println("Quay phải 90 độ...");

  while (abs(encoderCount) < targetPulses && millis() - startTime < timeout) {
    if (abs(encoderCount) >= targetPulses) break;
  }

  stopMotors();
  Serial.print("Hoàn thành quay phải 90 độ, Thời gian: "); 
  Serial.print(millis() - startTime); 
  Serial.print(" ms, Số xung: "); 
  Serial.println(abs(encoderCount));
  return true;
}

// HÀM QUAY 90 ĐỘ LÙI (XOAY TRÁI)
bool turn90DegreesBackward() {
  encoderCount = 0;
  unsigned long startTime = millis();
  int targetPulses = PULSES_FOR_90_DEG;
  const unsigned long timeout = 900;
  int motorSpeed = baseSpeed;

  Motor_left(0);
  Motor_right(motorSpeed);
  Serial.println("Quay trái 90 độ...");

  while (abs(encoderCount) < targetPulses && millis() - startTime < timeout) {
    if (abs(encoderCount) >= targetPulses) break;
  }

  stopMotors();
  Serial.print("Hoàn thành quay trái 90 độ, Thời gian: "); 
  Serial.print(millis() - startTime); 
  Serial.print(" ms, Số xung: "); 
  Serial.println(abs(encoderCount));
  return true;
}

bool moveForwardTime(unsigned long duration) {
  unsigned long startTime = millis();
  encoderCount = 0;
  int targetPulses = PULSES_FOR_10CM;

  moveForward();
  Serial.println("Tiến thẳng...");

  while (millis() - startTime < duration && abs(encoderCount) < targetPulses) {
    if (abs(encoderCount) >= targetPulses) break;
  }

  stopMotors();
  Serial.print("Hoàn thành tiến thẳng, Thời gian: "); 
  Serial.print(millis() - startTime); 
  Serial.print(" ms, Số xung: "); 
  Serial.println(abs(encoderCount));
  return true;
}

// CẢM BIẾN DÒ LINE
void lineSensorSetup() {
  pinMode(LINE_OUT1, INPUT);
  pinMode(LINE_OUT2, INPUT);
  pinMode(LINE_OUT3, INPUT);
  pinMode(LINE_OUT4, INPUT);
  pinMode(LINE_OUT5, INPUT);
}

// CẢM BIẾN SIÊU ÂM
void ultrasonicSetup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

float getDistance() {
  static float lastValidDistance = 999;
  static unsigned long lastMeasureTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastMeasureTime < 20) return lastValidDistance;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.034 / 2;

  if (duration == 0 || distance < 3 || distance > 34) return lastValidDistance;

  lastValidDistance = distance;
  lastMeasureTime = currentTime;
  return distance;
}

int obstacleState = 0;
unsigned long stateStartTime = 0;

// Hàm phụ để xử lý chờ
bool wait(int duration) {
  if (millis() - stateStartTime >= duration) {
    stopMotors();
    return true;
  }
  return false;
}

// XỬ LÝ DÒ LINE
float getLinePosition(int &lineStatus, int &s1, int &s2, int &s3, int &s4, int &s5) {
  int sum;
  readLineSensors(s1, s2, s3, s4, s5, sum);

  if (sum == 5) { // Tất cả cảm biến mất line
    lineStatus = 1;
    return lastLinePosition; // Trả về giá trị cuối cùng thay vì -1
  } else if (sum == 0) { // Tất cả cảm biến trên line
    lineStatus = 2;
    return lastLinePosition; // Trả về giá trị cuối cùng thay vì -1
  } else {
    lineStatus = 0;
    float linePosition = (s1 * -2.0 + s2 * -1.0 + s3 * 0.0 + s4 * 1.0 + s5 * 2.0) / (float)(5 - sum);
    lastLinePosition = linePosition; // Cập nhật lastLinePosition
    return linePosition;
  }
}

void followLine() {
  if (mode != 1) return;

  static bool stateInitialized = false;
  static unsigned long stateStartTime = 0;
  static unsigned long avoidStartTime = 0;
  static int obstacleState = 0;

  float distance = getDistance();

  if (millis() - lastAvoidTime > 2000 && distance >= 1.0 && distance <= 7.0 && distance > 0) {
    stopMotors();
    isAvoidingObstacle = true;
    obstacleState = 1;
    stateInitialized = false;
    Serial.println("Phát hiện vật cản, né tránh");
  }

  if (isAvoidingObstacle) {
    if (!stateInitialized) {
      stateStartTime = millis();
      avoidStartTime = millis();
      stateInitialized = true;
    }

    // if (millis() - avoidStartTime > 10000) {
    //   stopMotors();
    //   isAvoidingObstacle = false;
    //   obstacleState = 0;
    //   stateInitialized = false;
    //   Serial.println("Timeout né tránh, quay về dò line");
    //   return;
    // }

    switch (obstacleState) {
      case 1:
       stopMotors();
       delay (300);
        moveBackward();
        delay(300);
        obstacleState = 2;
        stateInitialized = false;
        break;
      case 2:
      stopMotors();
      delay(300);
        turnLeft();
        delay(280);
        obstacleState = 3;
        stateInitialized = false;
        break;
      case 3:
      stopMotors();
      delay(300);
        moveForwardTime(1800);
        obstacleState = 4;
        stateInitialized = false;
        break;
      case 4:
      stopMotors();
      delay(300);
        turnRight();
        delay(460);
        obstacleState = 5;
        stateInitialized = false;
        break;
      case 5:
        moveForward();
        if (millis() - stateStartTime > 100) {
          int s1, s2, s3, s4, s5, sum;
          readLineSensors(s1, s2, s3, s4, s5, sum);

          if (sum < 5) {
            stopMotors();
            isAvoidingObstacle = false;
            obstacleState = 0;
            mode = 1;
            lastAvoidTime = millis();
            stateInitialized = false;
            Serial.println("Né xong");
          }
        }
        break;
    }
    return;
  }

  int lineStatus = 1;
  int s1, s2, s3, s4, s5;
  getLinePosition(lineStatus, s1, s2, s3, s4, s5);
  int sum;
  readLineSensors(s1, s2, s3, s4, s5, sum);

  if (lineStatus == 2) {
    Serial.print("Line Status: 2 | S1: "); Serial.print(s1);
    Serial.print(" | S2: "); Serial.print(s2);
    Serial.print(" | S3: "); Serial.print(s3);
    Serial.print(" | S4: "); Serial.print(s4);
    Serial.print(" | S5: "); Serial.print(s5);
    Serial.println(" | Tất cả trên line, dừng xe");
    stopMotors();
    return;
  }

if (lineStatus == 1) {
    Serial.print("Line Status: 1 | S1: "); Serial.print(s1);
    Serial.print(" | S2: "); Serial.print(s2);
    Serial.print(" | S3: "); Serial.print(s3);
    Serial.print(" | S4: "); Serial.print(s4);
    Serial.print(" | S5: "); Serial.print(s5);
    Serial.println(" | Mất line, bắt đầu tìm line");

    unsigned long lastLineCheck = 0;
    const unsigned long lineCheckInterval = 10;
    bool lineFound = false;

    // Kiểm tra nếu lần cuối cùng S1 hoặc S5 phát hiện line
    Serial.print("lastLinePosition: "); Serial.println(lastLinePosition);

    // Chọn hướng quay dựa trên lastLinePosition
    if (lastLinePosition <= -1.5) { // S1 hoặc gần S1
        Serial.println("Quay phải để tìm line...");
        turnRight();
    } else if (lastLinePosition >= 1.5) { // S5 hoặc gần S5
        Serial.println("Quay trái để tìm line...");
        turnLeft();
    } else {
        // Mặc định quay trái nếu lastLinePosition không rõ ràng
        Serial.println("không rõ ràng , đi thẳng.");
        moveForward();
    }

    // Kiểm tra line trong khi quay
    while (!lineFound) {
        if (millis() - lastLineCheck >= lineCheckInterval) {
            readLineSensors(s1, s2, s3, s4, s5, sum);

            if (sum < 5) {
                Serial.print("Line Status: 0 | S1: "); Serial.print(s1);
                Serial.print(" | S2: "); Serial.print(s2);
                Serial.print(" | S3: "); Serial.print(s3);
                Serial.print(" | S4: "); Serial.print(s4);
                Serial.print(" | S5: "); Serial.print(s5);
                Serial.println(" tiếp tục bám line");
                stopMotors();
                lineFound = true;
            }
            lastLineCheck = millis();
        }
    }

    return;
}
  int leftSpeed = 0;
  int rightSpeed = 0;

  if (s3 == 0) {
    if (s1 == 0 && s2 == 0 && s3 == 0||s1 == 0 && s2 == 0) {
      Serial.print("Line Status: 0 | S1: "); Serial.print(s1);
      Serial.print(" | S2: "); Serial.print(s2);
      Serial.print(" | S3: "); Serial.print(s3);
      Serial.print(" | S4: "); Serial.print(s4);
      Serial.print(" | S5: "); Serial.print(s5);
      Serial.println("Góc phải, xoay phải 90 độ");
      stopMotors();
      delay(1000);
      turn90DegreesBackward();
      return;
    } else if (s3 == 0 && s4 == 0 && s5 == 0||s4 == 0 && s5 == 0) {
      Serial.print("Line Status: 0 | S1: "); Serial.print(s1);
      Serial.print(" | S2: "); Serial.print(s2);
      Serial.print(" | S3: "); Serial.print(s3);
      Serial.print(" | S4: "); Serial.print(s4);
      Serial.print(" | S5: "); Serial.print(s5);
      Serial.println("Góc trái, xoay trái 90 độ");
      stopMotors();
      delay(1000);
      turn90Degrees();
      return;
    } else {
      leftSpeed = baseSpeed;
      rightSpeed = baseSpeed;
    }
  } else if (s4 == 0||s5==0) {
    leftSpeed = baseSpeed;
    rightSpeed = 0;
  } else if (s2 == 0||s1==0) {
    leftSpeed = 0;
    rightSpeed = baseSpeed;
  }
  else if (s2 == 0 && s3 ==0){
    leftSpeed = baseSpeed;
    rightSpeed = 0;
  }
  else if (s3 ==0 || s4 ==0){
    leftSpeed = 0;
    rightSpeed = baseSpeed;
  }
  else{
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
  }

  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  distance = getDistance();
  if (distance < OBSTACLE_DISTANCE && distance > 0) {
    stopMotors();
    isAvoidingObstacle = true;
    obstacleState = 0;
    stateInitialized = false;
    Serial.println("né tránh");
    return;
  }

  Motor_left(leftSpeed);
  Motor_right(rightSpeed);

  unsigned long currentTime = millis();
  if (currentTime - lastLogTime >= logInterval) {
    Serial.print("Mode: "); Serial.print(mode);
    Serial.print(" | Line Status: "); Serial.print(lineStatus);
    Serial.print(" | S1: "); Serial.print(s1);
    Serial.print(" | S2: "); Serial.print(s2);
    Serial.print(" | S3: "); Serial.print(s3);
    Serial.print(" | S4: "); Serial.print(s4);
    Serial.print(" | S5: "); Serial.print(s5);
    Serial.print(" | Left Speed: "); Serial.print(leftSpeed);
    Serial.print(" | Right Speed: "); Serial.print(rightSpeed);
    Serial.print(" | Distance: "); Serial.print(distance);
    Serial.println(" cm");
    lastLogTime = currentTime;
  }
}

// ĐIỀU KHIỂN TAY
void manualControl() {
  if (mode != 0) return;

  if (data.buttonA) {
    moveForward();
    Serial.println("Nút A: Tiến");
  } else if (data.buttonB) {
    turnRight();
    Serial.println("Nút B: Xoay phải");
  } else if (data.buttonC) {
    moveBackward();
    Serial.println("Nút C: Lùi");
  } else if (data.buttonD) {
    turnLeft();
    Serial.println("Nút D: Xoay trái");
  } else {
    stopMotors();
    Serial.println("Dừng");
  }
}

// LED BÁO CHUYỂN MODE
void ledSetup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// CHÍNH
void setup() {
  Serial.begin(115200);
  motorSetup();
  ultrasonicSetup();
  encoderSetup();
  ledSetup();
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_2MBPS);
  radio.setRetries(1, 1);
  radio.startListening();
  Serial.println("Đang chờ kết nối...");
}

void loop() {
  if (!isConnected) {
    if (radio.available()) {
      radio.read(&data, sizeof(DataPacket));
      if (data.ping) {
        Serial.println("Nhận tín hiệu, phản hồi...");
        radio.stopListening();
        radio.write(&data, sizeof(DataPacket));
        radio.startListening();
        isConnected = true;
        Serial.println("Đã kết nối thành công !");
      }
    }
  } else {
    if (radio.available()) {
      radio.read(&data, sizeof(DataPacket));
      unsigned long currentTime = millis();
      if (currentTime - lastLogTime >= logInterval) {
        Serial.print("Joystick X: "); Serial.print(data.joystickX);
        Serial.print(" | Joystick Y: "); Serial.print(data.joystickY);
        Serial.print(" | A: "); Serial.print(data.buttonA);
        Serial.print(" | B: "); Serial.print(data.buttonB);
        Serial.print(" | C: "); Serial.print(data.buttonC);
        Serial.print(" | D: "); Serial.print(data.buttonD);
        Serial.print(" | E: "); Serial.print(data.buttonE);
        Serial.print(" | F: "); Serial.println(data.buttonF);
        lastLogTime = currentTime;
      }

      if (data.buttonE && mode != 1) {
        mode = 1;
        Serial.println(" chế độ tự động");
        blinkLED(1);
        lineSensorSetup();
      }
      if (data.buttonF && mode != 0) {
        mode = 0;
        Serial.println("chế độ điều khiển");
        blinkLED(2);
      }

      if (mode == 1) followLine();
      else if (mode == 0) manualControl();
    }
  }
}
