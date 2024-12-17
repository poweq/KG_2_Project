//by paul
#include <Encoder.h>
#include "motor.h"

// 엔코더 정의 (핀 번호는 기존 핀을 유지)
Encoder encoderA(18, 31);
Encoder encoderB(19, 38);
Encoder encoderC(3, 49);
Encoder encoderD(2, 55);

// dir 1 pin, dir 2 pin , pwm pin
Motor motorA(34, 35, 12);
Motor motorB(37, 36, 8);
Motor motorC(43, 42, 6);
Motor motorD(58, 59, 5);

// 모터 PWM 값 저장
int motorAValue = 0;
int motorBValue = 0;
int motorCValue = 0;
int motorDValue = 0;

// 엔코더 위치 저장
long encoderAPosition = 0;
long encoderBPosition = 0;
long encoderCPosition = 0;
long encoderDPosition = 0;

// 타이머 변수
unsigned long previousMillis = 0;
const long interval = 10;  // 1초

void setup() {
  Serial.begin(9600);  
  Serial3.begin(9600); 
}

void loop() {
  // 블루투스 모듈로부터 명령어 읽기

  if (Serial3.available() > 0) {
    String command = Serial3.readStringUntil('\n'); // 한 줄 읽기
    handleCommand(command);
  }

  // 시리얼 0번으로부터 명령어 읽기
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // 한 줄 읽기
    Serial.print(command);
    handleCommand(command);
  }

  // 엔코더 값을 읽기
  encoderAPosition = encoderA.read();
  encoderBPosition = encoderB.read();
  encoderCPosition = encoderC.read();
  encoderDPosition = encoderD.read();

  // 현재 시간 가져오기
  unsigned long currentMillis = millis();
  
  // 1초마다 엔코더 값을 0으로 초기화
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    encoderA.write(0);
    encoderB.write(0);
    encoderC.write(0);
    encoderD.write(0);
  }

  // 각 모터별 엔코더 값 출력 (시리얼 플로터용)
// 각 모터별 엔코더 값 출력 (디버깅용)
/*
Serial.print("A:");
Serial.print(abs(encoderAPosition));  // 절대값으로 출력
Serial.print(" B:");
Serial.print(abs(encoderBPosition));  // 절대값으로 출력
Serial.print(" C:");
Serial.print(abs(encoderCPosition));  // 절대값으로 출력
Serial.print(" D:");
Serial.println(abs(encoderDPosition)); // 절대값으로 출력

  delay(10);
*/
}

void handleCommand(String command) {
    //Serial.print("받은 명령: ");
    //Serial.println(command); // 받은 명령어 시리얼 모니터에 출력

    // W: 명령어 처리
    if (command.startsWith("W:")) {
        int value = command.substring(2).toInt();  // W: 뒤의 값을 정수로 변환
        motorAValue = value;
        motorBValue = value;
        motorCValue = value;
        motorDValue = value;
    } else {
        // 각 모터의 새로운 PWM 값을 설정
        updateMotorValue(command, 'a', motorAValue);
        updateMotorValue(command, 'b', motorBValue);
        updateMotorValue(command, 'c', motorCValue);
        updateMotorValue(command, 'd', motorDValue);
    }

    // 모터의 방향 및 PWM 값 설정
    setMotorDirectionAndPWM('a', motorAValue * 1.0);
    setMotorDirectionAndPWM('b', motorBValue * 0.9999999);
    setMotorDirectionAndPWM('c', motorCValue * 1.01);
    setMotorDirectionAndPWM('d', motorDValue * 1.02);

    // 출력값을 시리얼 0번으로 출력
   /*
    Serial.print("모터 출력: a:");
    Serial.print(motorAValue);
    Serial.print(" b:");
    Serial.print(motorBValue);
    Serial.print(" c:");
    Serial.print(motorCValue);
    Serial.print(" d:");
    Serial.println(motorDValue);
    */
}

void updateMotorValue(String command, char motor, int &motorValue) {
  int value = getMotorValue(command, motor);
  if (value != -1) {
    motorValue = value;  // 값이 존재할 경우에만 업데이트
  }
}

void setMotorDirectionAndPWM(char motor, int pwmValue) {
  int direction = (pwmValue >= 0) ? 1 : 0;  // PWM 값이 양수면 정방향, 음수면 역방향
  int absPWMValue = abs(pwmValue);  // PWM 값은 절대값으로 설정
  
  if (motor == 'a') {
    motorA.forward(pwmValue);
  } else if (motor == 'b') {
    motorB.forward(pwmValue);
  } else if (motor == 'c') {
    // 모터 C의 방향 반전
    motorC.forward(pwmValue);
  } else if (motor == 'd') {
    // 모터 D의 방향 반전
    motorD.forward(pwmValue);
    
  }
}

int getMotorValue(String command, char motor) {
  int index = command.indexOf(motor);
  if (index != -1) {
    int colonIndex = command.indexOf(':', index);
    if (colonIndex != -1) {
      int endIndex = command.indexOf(' ', colonIndex);
      if (endIndex == -1) endIndex = command.length();
      return command.substring(colonIndex + 1, endIndex).toInt();
    }
  }
  return -1;  // 값을 찾지 못했을 때는 -1 반환
}