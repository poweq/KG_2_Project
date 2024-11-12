#include <ESP32Servo.h> // ESP32 전용 Servo 라이브러리

Servo myServo; // 서보 모터 객체 생성
int servoPin = 18; // ESP32에서 서보 모터가 연결된 핀 번호 (예: GPIO 18)

void setup() {
  myServo.attach(servoPin); // 서보 모터 핀 연결
  myServo.write(0); // 초기 각도를 0도로 설정
  Serial.begin(115200); // 시리얼 통신 시작 (ESP32는 보통 115200 보레이트를 사용)
}

void loop() {
  if (Serial.available() > 0) { // 시리얼 입력이 있을 경우
    char command = Serial.read(); // 시리얼로 받은 문자 읽기

    if (command == 't') { // 입력이 't'인 경우
      myServo.write(0); // 서보 각도 0도로 설정
      Serial.println("Servo angle set to 0 degrees");
    }
    else if (command == 'f') { // 입력이 'f'인 경우
      myServo.write(90); // 서보 각도 90도로 설정
      Serial.println("Servo angle set to 90 degrees");
    }
  }
}
