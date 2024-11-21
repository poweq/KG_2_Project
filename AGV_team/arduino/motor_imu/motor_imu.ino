#include <Encoder.h>
#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h"

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

// 타이머 변수
unsigned long previousMillis = 0;
const long interval = 10;  // 10ms 간격

// IMU 관련 변수
MPU9250 mpu;
int16_t mpu_9250_raw_data[10];
void print_motor();
void print_imu();

// 데이터 수신 관련 변수
const int ARRAY_SIZE = 5;
int dataArray[ARRAY_SIZE];
String inputString = "";

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  Wire.begin();

  // MPU9250 초기화
  if (!mpu.setup(0x68)) {
    Serial.println("MPU9250 초기화 실패");
    while (1);
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

  // 시리얼 0번으로부터 명령어 읽기
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // 한 줄 읽기
    Serial.print(command);
    // handleCommand(command);
  }

  // 엔코더 값을 읽기
  encoderAPosition = encoderA.read();
  encoderBPosition = encoderB.read();
  encoderCPosition = encoderC.read();
  encoderDPosition = encoderD.read();

  // 현재 시간 가져오기
  unsigned long currentMillis = millis();
  
  // 10ms 간격으로 IMU 데이터와 엔코더 값을 전송
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendIMUData();
    encoderA.write(0);
    encoderB.write(0);
    encoderC.write(0);
    encoderD.write(0);
  }
  print_motor();
  delay(10);
}

void print_motor()
{
      // 출력값을 시리얼 0번으로 출력
    Serial.print("모터 출력: a:");
    Serial.print(motorAValue);
    Serial.print(" b:");
    Serial.print(motorBValue);
    Serial.print(" c:");
    Serial.print(motorCValue);
    Serial.print(" d:");
    Serial.println(motorDValue);
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

    // 모터의 방향 및 PWM 값 설정
    setMotorDirectionAndPWM('a', motorAValue);
    setMotorDirectionAndPWM('b', motorBValue);
    setMotorDirectionAndPWM('c', motorCValue);
    setMotorDirectionAndPWM('d', motorDValue);


  }
  else if (input.startsWith("241")) { // 0xf1의 10진수 표현인 241로 시작하는지 확인
    //Serial.println("241 명령 수신");

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
    if(speed <=150)
    {
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
        }

    // 모터의 방향 및 PWM 값 설정
    setMotorDirectionAndPWM('a', motorAValue);
    setMotorDirectionAndPWM('b', motorBValue);
    setMotorDirectionAndPWM('c', motorCValue);
    setMotorDirectionAndPWM('d', motorDValue);

  }
  else if (input.startsWith("242")) { // 0xf2의 10진수 표현인 242로 시작하는지 확인
    char direction = input[3]; // 세 번째 문자 가져오기
    int speed = 100;           // 좌우 이동 시 기본 속도 설정

    if (direction == 'l') {
      // 좌측 이동
      motorAValue = -speed; // 모터 A 후진
      motorBValue = speed;  // 모터 B 전진
      motorCValue = speed;  // 모터 C 전진
      motorDValue = -speed; // 모터 D 후진
    } else if (direction == 'r') {
      // 우측 이동
      motorAValue = speed;  // 모터 A 전진
      motorBValue = -speed; // 모터 B 후진
      motorCValue = -speed; // 모터 C 후진
      motorDValue = speed;  // 모터 D 전진
    } else {
      Serial.println("유효하지 않은 방향입니다: " + String(direction));
      return;
    }

    // 모터의 방향 및 PWM 값 설정
    setMotorDirectionAndPWM('a', motorAValue);
    setMotorDirectionAndPWM('b', motorBValue);
    setMotorDirectionAndPWM('c', motorCValue);
    setMotorDirectionAndPWM('d', motorDValue);
  }
}

void sendIMUData() {
  int16_t accX, accY, accZ;
  int16_t gyroX, gyroY, gyroZ;
  int16_t magX, magY, magZ;

  // 가속도계 및 자이로스코프 raw 데이터 읽기
  readAccelGyroRaw(accX, accY, accZ, gyroX, gyroY, gyroZ);

  // 자력계 raw 데이터 읽기
  bool magReadSuccess = readMagRaw(magX, magY, magZ);

  // 배열에 데이터 저장 (정수 값만)
  mpu_9250_raw_data[0] = 0xf3;
  mpu_9250_raw_data[1] = gyroX;
  mpu_9250_raw_data[2] = gyroY;
  mpu_9250_raw_data[3] = gyroZ;
  mpu_9250_raw_data[4] = accX;
  mpu_9250_raw_data[5] = accY;
  mpu_9250_raw_data[6] = accZ;
  if (magReadSuccess) {
    mpu_9250_raw_data[7] = magX;
    mpu_9250_raw_data[8] = magY;
    mpu_9250_raw_data[9] = magZ;
  } else {
    mpu_9250_raw_data[7] = 0;
    mpu_9250_raw_data[8] = 0;
    mpu_9250_raw_data[9] = 0;
  }

  // 배열 데이터를 시리얼 포트를 통해 전송
  Serial3.write((uint8_t*)mpu_9250_raw_data, sizeof(mpu_9250_raw_data));
  
  print_imu();
  
}
void print_imu()
{
  Serial.print("Gyro: ");
  Serial.print(mpu_9250_raw_data[1]);
  Serial.print(", ");
  Serial.print(mpu_9250_raw_data[2]);
  Serial.print(", ");
  Serial.print(mpu_9250_raw_data[3]);
  Serial.print(" | Accel: ");
  Serial.print(mpu_9250_raw_data[4]);
  Serial.print(", ");
  Serial.print(mpu_9250_raw_data[5]);
  Serial.print(", ");
  Serial.print(mpu_9250_raw_data[6]);
  Serial.print(" | Mag: ");
  Serial.print(mpu_9250_raw_data[7]);
  Serial.print(", ");
  Serial.print(mpu_9250_raw_data[8]);
  Serial.print(", ");
  Serial.println(mpu_9250_raw_data[9]);
}

// 가속도계 및 자이로스코프 raw 데이터 읽기
void readAccelGyroRaw(int16_t &accX, int16_t &accY, int16_t &accZ, int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ) {
  uint8_t raw_data[14];
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // ACCEL_XOUT_H 레지스터 시작 주소
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14);

  if (Wire.available() == 14) {
    // 가속도계 데이터 읽기
    accX = (Wire.read() << 8) | Wire.read();
    accY = (Wire.read() << 8) | Wire.read();
    accZ = (Wire.read() << 8) | Wire.read();

    // 온도 데이터 (사용하지 않음)
    Wire.read(); Wire.read();

    // 자이로스코프 데이터 읽기
    gyroX = (Wire.read() << 8) | Wire.read();
    gyroY = (Wire.read() << 8) | Wire.read();
    gyroZ = (Wire.read() << 8) | Wire.read();
  }
}

// 자력계 raw 데이터 읽기
bool readMagRaw(int16_t &magX, int16_t &magY, int16_t &magZ) {
  for (int attempt = 0; attempt < 5; attempt++) {  // 최대 5번 시도
    Wire.beginTransmission(0x0C);
    Wire.write(0x02); // AK8963_ST1 레지스터
    Wire.endTransmission(false);
    Wire.requestFrom(0x0C, 1);

    if (Wire.available() && (Wire.read() & 0x01)) {
      uint8_t raw_data[7];
      Wire.beginTransmission(0x0C);
      Wire.write(0x03); // AK8963_XOUT_L 레지스터
      Wire.endTransmission(false);
      Wire.requestFrom(0x0C, 7);

      if (Wire.available() == 7) {
        // 자력계 데이터 읽기
        magX = (Wire.read() | (Wire.read() << 8));
        magY = (Wire.read() | (Wire.read() << 8));
        magZ = (Wire.read() | (Wire.read() << 8));

        uint8_t overflow = Wire.read(); // ST2 레지스터 (오버플로우 플래그 확인)

        if (!(overflow & 0x08)) {
          return true; // 정상적으로 자력계 데이터 읽음
        }
      }
    }
    delay(10); // 데이터 준비 시간을 위해 지연
  }
  return false; // 데이터 읽기 실패
}

void setMotorDirectionAndPWM(char motor, int pwmValue) {
  int direction = (pwmValue >= 0) ? 1 : 0;  // PWM 값이 양수면 정방향, 음수면 역방향
  int absPWMValue = abs(pwmValue);  // PWM 값은 절대값으로 설정
  
  if (motor == 'a') {
    digitalWrite(motorA_DirectionPin1, direction);
    digitalWrite(motorA_DirectionPin2, !direction);
    analogWrite(motorA_PWM_Pin, absPWMValue);
  } else if (motor == 'b') {
    digitalWrite(motorB_DirectionPin1, direction);
    digitalWrite(motorB_DirectionPin2, !direction);
    analogWrite(motorB_PWM_Pin, absPWMValue);
  } else if (motor == 'c') {
    // 모터 C의 방향 반전
    digitalWrite(motorC_DirectionPin1, !direction);
    digitalWrite(motorC_DirectionPin2, direction);
    analogWrite(motorC_PWM_Pin, absPWMValue);
  } else if (motor == 'd') {
    // 모터 D의 방향 반전
    digitalWrite(motorD_DirectionPin1, !direction);
    digitalWrite(motorD_DirectionPin2, direction);
    analogWrite(motorD_PWM_Pin, absPWMValue);
  }
}
