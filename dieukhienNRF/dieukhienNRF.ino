#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // CE: A3 (17), CSN: D10 (10)

const byte address[6] = "00001"; // Địa chỉ kênh giao tiếp
bool isConnected = false; // Trạng thái kết nối
unsigned long lastSendTime = 0; // Thời gian gửi lần cuối
unsigned long lastLogTime = 0; // Thời gian log lần cuối
const unsigned long sendInterval = 10; // Gửi mỗi 10ms (100Hz)
const unsigned long logInterval = 100; // Log mỗi 100ms

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

void setup() {
  Serial.begin(115200); // Tăng baud rate
  radio.begin();
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address); // Để nhận phản hồi từ xe
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_2MBPS); // Tăng tốc độ truyền lên 2Mbps
  radio.setRetries(1, 1); // Giảm số lần thử lại để giảm độ trễ

  // Khai báo chân joystick và nút bấm
  pinMode(A0, INPUT); // Joystick X
  pinMode(A1, INPUT); // Joystick Y
  pinMode(2, INPUT_PULLUP); // Nút A
  pinMode(3, INPUT_PULLUP); // Nút B
  pinMode(4, INPUT_PULLUP); // Nút C
  pinMode(5, INPUT_PULLUP); // Nút D
  pinMode(6, INPUT_PULLUP); // Nút E
  pinMode(7, INPUT_PULLUP); // Nút F
}

void loop() {
  // Đọc dữ liệu từ joystick và nút bấm
  data.joystickX = analogRead(A0);
  data.joystickY = analogRead(A1);
  data.buttonA = !digitalRead(2);
  data.buttonB = !digitalRead(3);
  data.buttonC = !digitalRead(4);
  data.buttonD = !digitalRead(5);
  data.buttonE = !digitalRead(6);
  data.buttonF = !digitalRead(7);

  unsigned long currentTime = millis();

  if (!isConnected) {
    // Gửi tín hiệu ping để kiểm tra kết nối
    if (currentTime - lastSendTime >= sendInterval) {
      data.ping = true;
      radio.stopListening();
      if (radio.write(&data, sizeof(DataPacket))) {
        Serial.println("Đang kết nối với xe...");
      } else {
        Serial.println("Gửi tín hiệu ping thất bại!");
      }
      radio.startListening();
      lastSendTime = currentTime;

      // Kiểm tra phản hồi ngay lập tức
      if (radio.available()) {
        radio.read(&data, sizeof(DataPacket));
        if (data.ping) {
          isConnected = true;
          radio.stopListening(); // Sau khi kết nối, luôn ở chế độ gửi
          Serial.println("Đã kết nối thành công với xe!");
        }
      }
    }
  } else {
    // Khi đã kết nối, gửi dữ liệu điều khiển
    if (currentTime - lastSendTime >= sendInterval) {
      data.ping = false;
      if (radio.write(&data, sizeof(DataPacket))) {
        // Log dữ liệu với tần suất thấp hơn
        if (currentTime - lastLogTime >= logInterval) {
          Serial.print("X: "); Serial.print(data.joystickX);
          Serial.print(" | Y: "); Serial.print(data.joystickY);
          Serial.print(" | A: "); Serial.print(data.buttonA);
          Serial.print(" | B: "); Serial.print(data.buttonB);
          Serial.print(" | C: "); Serial.print(data.buttonC);
          Serial.print(" | D: "); Serial.print(data.buttonD);
          Serial.print(" | E: "); Serial.print(data.buttonE);
          Serial.print(" | F: "); Serial.println(data.buttonF);
          lastLogTime = currentTime;
        }
      } else {
        Serial.println("Gửi dữ liệu thất bại!");
      }
      lastSendTime = currentTime;
    }
  }
}