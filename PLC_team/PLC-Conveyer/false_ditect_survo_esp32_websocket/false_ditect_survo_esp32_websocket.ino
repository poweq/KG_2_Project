#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>  // ESP32에서 서보모터 제어를 위한 라이브러리

#define ssid  "ConnectValue_A401_2G"
#define password   "CVA401!@#$"

WebSocketsServer webSocket = WebSocketsServer(80);

// 서보모터 핀 설정
Servo myServo;
int servoPin = 18;  // 서보모터가 연결된 핀 번호

// WebSocket 이벤트 처리 함수
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
  // Figure out the type of WebSocket event
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;

    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connection from ", num);
        Serial.println(ip.toString());
      }
      break;

    case WStype_TEXT:
      Serial.printf("[%u] Text: %s\n", num, payload);
      
      // 수신된 텍스트에 따라 서보모터 제어
      if (payload[0] == 't') {
        Serial.println("Setting servo to 0 degrees.");
        myServo.write(90);  // 서보모터를 0도로 이동
      } 
      else if (payload[0] == 'f') {
        Serial.println("Setting servo to 90 degrees.");
        myServo.write(180);  // 서보모터를 90도로 이동
      }
      
      webSocket.sendTXT(num, payload);
      break;

    default:
      break;
  }
}

void setup() {
  // Start Serial port
  Serial.begin(115200);
  
  // Connect to Wi-Fi
  Serial.println("Connecting");
  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  }
  
  // Print our IP address
  Serial.println("Connected!");
  Serial.print("My IP address: ");
  Serial.println(WiFi.localIP());
  
  // 서보모터 핀 초기화
  myServo.attach(servoPin);
  
  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void loop() {
  // Look for and handle WebSocket data
  webSocket.loop();
}
