#include <Servo.h> // Servo 라이브러리 포함

Servo myServo;      // Servo 객체 생성
const int servoPin = 3; // 서보모터 핀 번호

void setup() {
  myServo.attach(servoPin); // 서보모터 핀 연결
  myServo.write(0);         // 초기 위치 설정 (0도)
  Serial.begin(9600);       // 시리얼 통신 시작
}

void loop() {
  if (Serial.available() > 0) { // 시리얼 데이터가 수신되었는지 확인
    char command = Serial.read(); // 명령어 읽기

    if (command == 't') {
      myServo.write(0); // 90도 설정
      Serial.println("Servo set to 90 degrees");
    } 
    else if (command == 'f') {
      myServo.write(40); // 45도 설정
      Serial.println("Servo set to 45 degrees");
    }
  }
}
