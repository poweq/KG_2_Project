// Motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
    int directionPin1;  // 직진 방향 핀
    int directionPin2;  // 후진 방향 핀
    int pwmPin;         // PWM 핀

public:
    // 생성자: 방향 핀과 PWM 핀을 설정
    Motor(int dirPin1, int dirPin2, int pwm);

    // 직진 함수
    void forward(int pwmValue);

    // 후진 함수
    void reverse(int pwmValue);

    // 모터 정지 함수
    void stop();
};

#endif
