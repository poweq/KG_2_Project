#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// WiFi 설정
const char* ssid = "kg_2";
const char* password = "kg123123";

// ROS2 브릿지 서버 설정
const char* rosbridge_server = "192.168.0.200";
const uint16_t rosbridge_port = 9090;

// WebSocket 객체 생성
WebSocketsClient webSocket; // 'webSocket' 객체가 제대로 선언되었습니다.

// ROS2 토픽 설정
const char* topic_status = "/case_status";
const char* topic_command = "/case_command"; // 구독할 토픽
const char* message_type = "std_msgs/String";

// 스텝 모터 핀 설정
const int stepXPin = 4;   // STEP 핀
const int dirXPin = 2;    // DIR 핀
const int enPin = 22;     // ENABLE 핀
const int irSensorPin = 34; // IR 센서 핀 (D34)

// 서보모터 핀 설정
const int servoPin = 23;  // 서보모터 연결 핀 (ESP32에서 PWM 핀)

// 스텝 모터 객체 생성
AccelStepper stepper(AccelStepper::DRIVER, stepXPin, dirXPin);
Servo myServo;  // 서보모터 객체 생성

// 동작 횟수 추적 변수
int motorCount = 0;

// 각도 저장 변수
int motorAngles[3] = {67, 134, 201}; // a:1, a:2, a:3 각도에 해당하는 스텝 값

// 상태 변수
bool isServoClosed = false; // 서보모터가 닫힌 상태인지 여부를 추적

void setup() {
  Serial.begin(115200);

  // WiFi 연결
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nROS2 connected");

  // WebSocket 연결 설정
  webSocket.begin(rosbridge_server, rosbridge_port, "/"); // webSocket 초기화
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket initialized");

  // 서보모터 초기화
  myServo.attach(servoPin);  // 서보모터 핀 설정

  // 스텝 모터 핀 및 초기 설정
  pinMode(enPin, OUTPUT);
  pinMode(irSensorPin, INPUT); // IR 센서를 입력 모드로 설정
  digitalWrite(enPin, LOW);    // 모터 활성화
  stepper.setMaxSpeed(1000);   // 최대 속도 (스텝/초)
  stepper.setAcceleration(500); // 가속도 (스텝/초^2)

  Serial.println("IR Sensor and Motor initialized. Waiting for detection...");
}

void loop() {
  // WebSocket 통신 유지
  webSocket.loop();

  // "close"일 때만 동작 수행
  if (isServoClosed) {
    // IR 센서 값 읽기
    int irValue = digitalRead(irSensorPin);

    // IR 센서가 물체를 감지하면 동작 수행
    if (irValue == LOW) {
      Serial.println("Object detected! Waiting 2 seconds");
      delay(2000); // 2초 대기

      // 모터 회전 (120도: 67 스텝)
      stepper.move(67);
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }

      // 동작 횟수 증가 및 메시지 결정
      motorCount = (motorCount % 3) + 1; // 1, 2, 3 반복
      String message = "a:" + String(motorCount);

      // ROS2 메시지 발행
      sendMessageToROS(topic_status, message.c_str());
    }
  }

  // 시리얼 입력 처리 (open, close)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // 입력받은 문자열을 읽음
    command.trim();  // 앞뒤 공백 제거

    if (command == "open") {
      Serial.println("Opening servo to 90 degrees.");
      myServo.write(90);  // 서보모터를 90도로 회전
      isServoClosed = false;  // 서보모터가 열려있는 상태
    } else if (command == "close") {
      Serial.println("Closing servo to 0 degrees.");
      myServo.write(0);   // 서보모터를 0도로 회전
      isServoClosed = true;  // 서보모터가 닫힌 상태
    } else {
      Serial.println("Invalid command. Use 'open' or 'close'.");
    }
  }
}

// WebSocket 이벤트 처리 함수
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.println("WebSocket connected");
    advertiseTopic(topic_status); // `/case_status` 토픽 등록 요청
    subscribeTopic(topic_command); // `/case_command` 토픽 구독 요청
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("WebSocket disconnected");
  } else if (type == WStype_TEXT) {
    // 수신된 메시지 처리
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
      Serial.print("JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }

    // 메시지의 토픽 및 내용 확인
    const char* topic = doc["topic"];
    const char* data = doc["msg"]["data"];

    if (strcmp(topic, topic_command) == 0) {
      Serial.printf("Message received on %s: %s\n", topic_command, data);

      // 메시지 내용에 따른 모터 제어
      if (strcmp(data, "a:1") == 0) {
        rotateMotorToAngle(motorAngles[0]);
        sendMessageToROS(topic_status, "a:1");
      } else if (strcmp(data, "a:2") == 0) {
        rotateMotorToAngle(motorAngles[1]);
        sendMessageToROS(topic_status, "a:2");
      } else if (strcmp(data, "a:3") == 0) {
        rotateMotorToAngle(motorAngles[2]);
        sendMessageToROS(topic_status, "a:3");
      } else if (strcmp(data, "a:open") == 0) {
        // "a:open" 메시지 수신 시 서보모터 열기
        Serial.println("Opening servo to 90 degrees from case_command");
        myServo.write(90);  // 서보모터를 90도로 회전
        isServoClosed = false;  // 서보모터가 열려있는 상태
      } else if (strcmp(data, "a:close") == 0) {
        // "a:close" 메시지 수신 시 서보모터 닫기
        Serial.println("Closing servo to 0 degrees from case_command");
        myServo.write(0);   // 서보모터를 0도로 회전
        isServoClosed = true;  // 서보모터가 닫힌 상태
      } else {
        Serial.printf("Unhandled command: %s\n", data);
      }
    }
  }
}

// ROS2 토픽 등록 (advertise)
void advertiseTopic(const char* topic_name) {
  StaticJsonDocument<256> doc;

  doc["op"] = "advertise";
  doc["topic"] = topic_name;
  doc["type"] = message_type;

  String jsonString;
  serializeJson(doc, jsonString);

  webSocket.sendTXT(jsonString);
  Serial.println("Topic advertised: " + jsonString);
}

// ROS2 토픽 구독 (subscribe)
void subscribeTopic(const char* topic_name) {
  StaticJsonDocument<256> doc;

  doc["op"] = "subscribe";
  doc["topic"] = topic_name;
  doc["type"] = message_type;

  String jsonString;
  serializeJson(doc, jsonString);

  webSocket.sendTXT(jsonString);
  Serial.println("Subscribed to topic: " + jsonString);
}

// ROS2 토픽에 메시지 보내기
void sendMessageToROS(const char* topic_name, const char* message) {
  StaticJsonDocument<256> doc;

  doc["op"] = "publish";
  doc["topic"] = topic_name;
  
  JsonObject msg = doc.createNestedObject("msg");
  msg["data"] = message; // std_msgs/String 형식의 "data" 필드

  String jsonString;
  serializeJson(doc, jsonString);

  webSocket.sendTXT(jsonString);
  Serial.println("Message sent to " + String(topic_name) + ": " + jsonString);
}

// 스텝 모터를 특정 각도로 회전시키는 함수
void rotateMotorToAngle(int steps) {
  Serial.printf("Rotating motor to %d steps...\n", steps);
  stepper.moveTo(steps);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  Serial.println("Motor rotation complete.");
}
