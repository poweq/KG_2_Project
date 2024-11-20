#include <Encoder.h>
#include <Arduino.h>
#include <MPU9250.h>

// MPU9250 객체 생성
MPU9250 mpu;

// 엔코더 정의 (핀 번호는 기존 핀을 유지)
Encoder encoderA(18, 31);
Encoder encoderB(19, 38);
Encoder encoderC(3, 49);
Encoder encoderD(2, 55);

// 모터 제어 핀 설정
const int motorA_DirectionPin1 = 34;
const int motorA_DirectionPin2 = 35;
const int motorA_PWM_Pin = 12;

const int motorB_DirectionPin1 = 37;
const int motorB_DirectionPin2 = 36;
const int motorB_PWM_Pin = 8;

const int motorC_DirectionPin1 = 43;
const int motorC_DirectionPin2 = 42;
const int motorC_PWM_Pin = 6;

const int motorD_DirectionPin1 = 58;
const int motorD_DirectionPin2 = 59;
const int motorD_PWM_Pin = 5;

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

// IMU 데이터 저장 변수
float gyroX, gyroY, gyroZ;
float accX, accY, accZ;

// 타이머 변수
unsigned long previousMillis = 0;
const long motorInterval = 10;  // 모터 제어 간격 (10ms)

unsigned long previousIMUMillis = 0;
const long imuInterval = 50;  // IMU 데이터 전송 간격 (10ms)

const int ARRAY_SIZE = 5;
int dataArray[ARRAY_SIZE];
String inputString = "";

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  Wire.begin();

  // MPU9250 초기화
  if (!mpu.setup(0x68)) { // I2C 주소 0x68로 MPU9250 설정
    Serial.println("MPU9250 초기화 실패");
    while (1); // 초기화 실패 시 무한 대기
  }
  Serial.println("MPU9250 초기화 성공");

  // 모터 핀을 출력으로 설정
  pinMode(motorA_DirectionPin1, OUTPUT);
  pinMode(motorA_DirectionPin2, OUTPUT);
  pinMode(motorA_PWM_Pin, OUTPUT);
  
  pinMode(motorB_DirectionPin1, OUTPUT);
  pinMode(motorB_DirectionPin2, OUTPUT);
  pinMode(motorB_PWM_Pin, OUTPUT);

  pinMode(motorC_DirectionPin1, OUTPUT);
  pinMode(motorC_DirectionPin2, OUTPUT);
  pinMode(motorC_PWM_Pin, OUTPUT);

  pinMode(motorD_DirectionPin1, OUTPUT);
  pinMode(motorD_DirectionPin2, OUTPUT);
  pinMode(motorD_PWM_Pin, OUTPUT);
}

void loop() {
  // 블루투스 모듈로부터 명령어 읽기
  while (Serial3.available() > 0) {
    char data = Serial3.read();
    if (data == '\n') { // 줄바꿈 문자를 만나면 데이터 처리를 시작
      processInput(inputString);
      inputString = ""; // 입력 문자열 초기화
    } else {
      inputString += data; // 입력 문자열에 데이터 추가
    }
  }

  // 현재 시간 가져오기
  unsigned long currentMillis = millis();

  // IMU 데이터를 10ms마다 읽고 전송
  if (currentMillis - previousIMUMillis >= imuInterval) {
    previousIMUMillis = currentMillis;

    // MPU9250 업데이트
    if (mpu.update()) {
      // IMU 데이터 읽기 및 저장
      gyroX = mpu.getGyroX();
      gyroY = mpu.getGyroY();
      gyroZ = mpu.getGyroZ();
      accX = mpu.getAccX();
      accY = mpu.getAccY();
      accZ = mpu.getAccZ();

      // 시리얼로 IMU 데이터 전송 (라즈베리 파이로 전송)
      Serial3.print("242,"); // 0xf2의 10진수 표현인 242를 첫 번째로 전송
      Serial3.print(gyroX);
      Serial3.print(",");
      Serial3.print(gyroY);
      Serial3.print(",");
      Serial3.print(gyroZ);
      Serial3.print(",");
      Serial3.print(accX);
      Serial3.print(",");
      Serial3.print(accY);
      Serial3.print(",");
      Serial3.println(accZ);
    }
  }

  // 모터 제어를 10ms마다 수행
  if (currentMillis - previousMillis >= motorInterval) {
    previousMillis = currentMillis;

    // 엔코더 값을 읽기
    encoderAPosition = encoderA.read();
    encoderBPosition = encoderB.read();
    encoderCPosition = encoderC.read();
    encoderDPosition = encoderD.read();

    // 모터의 방향 및 PWM 값 설정
    setMotorDirectionAndPWM('a', motorAValue);
    setMotorDirectionAndPWM('b', motorBValue);
    setMotorDirectionAndPWM('c', motorCValue);
    setMotorDirectionAndPWM('d', motorDValue);

    // 엔코더 값을 0으로 초기화
    encoderA.write(0);
    encoderB.write(0);
    encoderC.write(0);
    encoderD.write(0);
  }
}

void processInput(String input) {
  // 입력 문자열이 0xf0로 시작하는지 확인
  if (input.startsWith("240")) { // 0xf0의 10진수 표현인 240으로 시작하는지 확인
    // 입력 문자열을 ","로 분할
    int currentIndex = 0;
    int startIndex = 0;
    for (int i = 0; i < ARRAY_SIZE; i++) {
      currentIndex = input.indexOf(',', startIndex);
      if (currentIndex == -1) {
        currentIndex = input.length();
      }
      String valueString = input.substring(startIndex, currentIndex);
      dataArray[i] = valueString.toInt();
      startIndex = currentIndex + 1;
    }
    // 모터 PWM 값 설정 (노이즈 필터링 - 절대값이 100 이상인 경우 무시)
    if (abs(dataArray[1]) < 100) motorAValue = dataArray[1];
    if (abs(dataArray[2]) < 100) motorBValue = dataArray[2];
    if (abs(dataArray[3]) < 100) motorCValue = dataArray[3];
    if (abs(dataArray[4]) < 100) motorDValue = dataArray[4];

    // 출력값을 시리얼로 출력 (디버깅용)
    Serial.print("모터 출력: a:");
    Serial.print(motorAValue);
    Serial.print(" b:");
    Serial.print(motorBValue);
    Serial.print(" c:");
    Serial.print(motorCValue);
    Serial.print(" d:");
    Serial.println(motorDValue);
  }
  else if (input.startsWith("241")) { // 0xf1의 10진수 표현인 241로 시작하는지 확인
    Serial.println("241 명령 수신");

    // 입력 문자열을 ","로 분할하여 첫 번째, 두 번째, 세 번째 값을 가져옴
    int currentIndex = 0;
    int startIndex = 0;
    String tokens[3];
    int tokenIndex = 0;

    while (tokenIndex < 3) {
      currentIndex = input.indexOf(',', startIndex);
      if (currentIndex == -1) {
        currentIndex = input.length();
      }
      tokens[tokenIndex] = input.substring(startIndex, currentIndex);
      startIndex = currentIndex + 1;
      tokenIndex++;
    }

    // 방향 및 속도 값 설정
    String direction = tokens[1];
    int speed = tokens[2].toInt();

    if (direction == "cc") {
      // 반시계 방향 회전 설정
      motorAValue = -speed;  // 모터 A 후진
      motorBValue = -speed;   // 모터 B 전진
      motorCValue = speed;   // 모터 C 전진
      motorDValue = speed;  // 모터 D 후진
    } else if (direction == "cw") {
      // 시계 방향 회전 설정
      motorAValue = speed;   // 모터 A 전진
      motorBValue = speed;  // 모터 B 후진
      motorCValue = -speed;  // 모터 C 후진
      motorDValue = -speed;   // 모터 D 전진
    } else {
      // 유효하지 않은 방향 값일 경우 무시
      Serial.println("유효하지 않은 방향입니다: " + direction);
      return;
    }

    // 모터의 방향 및 PWM 값 설정
    setMotorDirectionAndPWM('a', motorAValue);
    setMotorDirectionAndPWM('b', motorBValue);
    setMotorDirectionAndPWM('c', motorCValue);
    setMotorDirectionAndPWM('d', motorDValue);

    // 출력값을 시리얼로 출력 (디버깅용)
    Serial.print("모터 출력: 방향:");
    Serial.print(direction);
    Serial.print(" 속도:");
    Serial.println(speed);
  }
}

void setMotorDirectionAndPWM(char motor, int pwmValue) {
  int direction = (pwmValue >= 0) ? HIGH : LOW;  // PWM 값이 양수면 정방향, 음수면 역방향
  int absPWMValue = abs(pwmValue);  // PWM 값은 절대값으로 설정
  absPWMValue = constrain(absPWMValue, 0, 255);  // PWM 값 제한 (0-255)

  if (motor == 'a') {
    digitalWrite(motorA_DirectionPin1, direction);
    digitalWrite(motorA_DirectionPin2, !direction);
    analogWrite(motorA_PWM_Pin, absPWMValue);
  } else if (motor == 'b') {
    digitalWrite(motorB_DirectionPin1, direction);
    digitalWrite(motorB_DirectionPin2, !direction);
    analogWrite(motorB_PWM_Pin, absPWMValue);
  } else if (motor == 'c') {
    digitalWrite(motorC_DirectionPin1, !direction); // 모터 C의 방향 반전
    digitalWrite(motorC_DirectionPin2, direction);
    analogWrite(motorC_PWM_Pin, absPWMValue);
  } else if (motor == 'd') {
    digitalWrite(motorD_DirectionPin1, !direction); // 모터 D의 방향 반전
    digitalWrite(motorD_DirectionPin2, direction);
    analogWrite(motorD_PWM_Pin, absPWMValue);
  }
}
