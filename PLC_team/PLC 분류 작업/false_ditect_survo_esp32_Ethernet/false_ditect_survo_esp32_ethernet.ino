#include <ESP32Servo.h>  // ESP32Servo 라이브러리 포함

Servo myservo;
int pos = 0;  // 서보 모터의 초기 각도

void setup() {
  Serial.begin(115200);  // 직렬 통신 시작
  myservo.attach(18);  // 서보 모터를 GPIO 13번에 연결 (필요에 맞게 변경)
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();  // 직렬로 받은 명령 읽기
    if (cmd == 't') {
      pos = 0;  // 0도 설정
      myservo.write(pos);  // 서보 모터 이동
    } else if (cmd == 'f') {
      pos = 90;  // 90도 설정
      myservo.write(pos);  // 서보 모터 이동
    }
  }
}
