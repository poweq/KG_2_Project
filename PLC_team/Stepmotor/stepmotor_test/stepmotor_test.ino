#include <AccelStepper.h>

// AccelStepper 초기화: Driver 모드 (STEP 핀, DIR 핀)
AccelStepper stepper(AccelStepper::DRIVER, 19, 18);

void setup() {
  Serial.begin(115200);

  // A4988 활성화 핀 설정
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW); // 모터 활성화

  // 스텝 모터 초기 설정
  stepper.setMaxSpeed(1000);   // 최대 속도 (스텝/초)
  stepper.setAcceleration(500); // 가속도 (스텝/초^2)
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'a') {
      // 90도 회전 (시계 방향)
      stepper.move(200); // 200 스텝 (1.8도 기준 90도)
    } else if (command == 'b') {
      // 0도로 회전 (반시계 방향)
      stepper.moveTo(0); // 원점으로 복귀
    }
  }

  // 스텝 모터 실행
  stepper.run();
}