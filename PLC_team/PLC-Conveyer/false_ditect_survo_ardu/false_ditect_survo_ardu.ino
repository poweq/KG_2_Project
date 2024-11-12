#include <Servo.h> // Servo 라이브러리 포함

Servo myServo; // 서보 모터 객체 생성
int servoPin = 8; // 서보 모터가 연결된 핀 번호

void setup() {
  myServo.attach(servoPin); // 서보 모터 핀 연결
  myServo.write(0); // 초기 각도를 0도로 설정
  Serial.begin(9600); // 시리얼 통신 시작 (보레이트 9600)
}

void loop() {
  if (Serial.available() > 0) { // 시리얼 입력이 있을 경우
    char command = Serial.read(); // 시리얼로 받은 문자 읽기

    if (command == 'a') { // 입력이 't'인 경우
      myServo.write(90); // 서보 각도 0도로 설정
      Serial.println("Servo angle set to 0 degrees");
    }
    else if (command == 'b') { // 입력이 'f'인 경우
      myServo.write(0); // 서보 각도 90도로 설정
      Serial.println("Servo angle set to 90 degrees");
    }
  }
}
