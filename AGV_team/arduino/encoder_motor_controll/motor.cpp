// Motor.cpp
#include "Motor.h"

Motor::Motor(int dirPin1, int dirPin2, int pwm)
    : directionPin1(dirPin1), directionPin2(dirPin2), pwmPin(pwm) {
    pinMode(directionPin1, OUTPUT);
    pinMode(directionPin2, OUTPUT);
    pinMode(pwmPin, OUTPUT);
}

void Motor::forward(int pwmValue) {
    digitalWrite(directionPin1, HIGH);  // 직진 방향 핀 활성화
    digitalWrite(directionPin2, LOW);   // 후진 방향 핀 비활성화
    analogWrite(pwmPin, pwmValue);      // PWM 출력
}

void Motor::reverse(int pwmValue) {
    digitalWrite(directionPin1, LOW);   // 직진 방향 핀 비활성화
    digitalWrite(directionPin2, HIGH);  // 후진 방향 핀 활성화
    analogWrite(pwmPin, pwmValue);      // PWM 출력
}

void Motor::stop() {
    digitalWrite(directionPin1, LOW);   // 방향 핀 비활성화
    digitalWrite(directionPin2, LOW);   // 방향 핀 비활성화
    analogWrite(pwmPin, 0);            // PWM 출력 0 (정지)
}
