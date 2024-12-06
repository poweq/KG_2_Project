const int enPin = 8;
const int stepXPin = 2; // X.STEP
const int dirXPin = 5;  // X.DIR

const int stepsPerRevolution = 200; // 스텝 모터의 1회전 당 스텝 수
int step_delay = 2000;  // 스텝 속도 조절용 딜레이 (마이크로초)

void setup() {
  Serial.begin(9600);  // 시리얼 통신 초기화
  pinMode(enPin, OUTPUT);
  pinMode(stepXPin, OUTPUT);
  pinMode(dirXPin, OUTPUT);

  digitalWrite(enPin, LOW); // 모터 드라이버 활성화
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();  // 시리얼 입력 읽기
    
    if (command == 'a') {
      int steps = stepsPerRevolution;  // 360도 = 1회전
      rotateMotor(stepXPin, dirXPin, HIGH, steps);  // X축 전진 회전
    } 
    else if (command == 'b') {
      int steps = stepsPerRevolution;  // 360도 = 1회전
      rotateMotor(stepXPin, dirXPin, LOW, steps);  // X축 반대 방향 회전
    } 
    else if (command == 's') {
      Serial.println("Enter speed (1-10):");
      while (!Serial.available()); // 속도 입력 대기
      int speed_input = Serial.parseInt();  // 속도 입력값 읽기
      if (speed_input >= 1 && speed_input <= 10) {
        step_delay = map(speed_input, 1, 10, 1000, 100); // 속도에 따라 딜레이 맵핑
        Serial.print("Speed set to: ");
        Serial.println(speed_input);
      } else {
        Serial.println("Invalid speed input");
      }
    } 
    else {
      Serial.println("Invalid command");
    }
  }
}

void rotateMotor(int stepPin, int dirPin, int direction, int steps) {
  digitalWrite(dirPin, direction);  // 회전 방향 설정
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(step_delay);  // 속도 조절용 딜레이
    digitalWrite(stepPin, LOW);
    delayMicroseconds(step_delay);  // 속도 조절용 딜레이
  }
}
