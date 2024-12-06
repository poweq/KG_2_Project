import cv2
import numpy as np
import websockets
import asyncio
import serial
from pyzbar.pyzbar import decode
import time
import json
import websocket

# 아두이노 연결 설정
arduino = serial.Serial('com11', 9600, timeout=1)

# 등록된 QR 코드 주소 리스트
valid_addresses = [
    "https://site.naver.com/patient/B_2",
    "https://example.com/address1",
    "https://example.com/address2"
    # 추가적으로 더 많은 주소를 여기에 넣을 수 있습니다.
]

# QR 코드 인식 여부를 제어하는 변수
qr_recognition_enabled = True

# ROS2WebSocketClient 클래스 정의
class ROS2WebSocketClient:
    def __init__(self, rosbridge_ip='192.168.0.200', rosbridge_port=9090):
        self.rosbridge_ip = rosbridge_ip
        self.rosbridge_port = rosbridge_port
        self.ws = None
        self.is_connected = False

    def connect(self):
        # 웹소켓 서버에 연결
        websocket.enableTrace(False)
        ws_url = f"ws://{self.rosbridge_ip}:{self.rosbridge_port}"

        self.ws = websocket.WebSocketApp(
            ws_url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        # 웹소켓 연결 시작
        self.ws.run_forever()

    def on_open(self, ws):
        self.is_connected = True
        print(f"Connected to ROS2 at {self.rosbridge_ip}:{self.rosbridge_port}")
        
        # 연결 후 주제 광고
        self.advertise_topic("/robot_arm_command", "std_msgs/String")
        
        # 연결 후 2초 뒤에 'start' 명령어 게시
        time.sleep(2)
        self.publish_message("/robot_arm_command", {"data": "start"})  # "start" 메시지 보내기

    def on_message(self, ws, message):
        # 서버로부터 받은 메시지 처리
        print(f"Received message: {message}")

    def on_error(self, ws, error):
        print(f"Error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        self.is_connected = False
        print(f"Connection closed: {close_msg}")

    def advertise_topic(self, topic, msg_type):
        # ROS2의 주제를 광고
        advertise_message = {
            "op": "advertise",
            "topic": topic,
            "type": msg_type
        }
        self.ws.send(json.dumps(advertise_message))
        print(f"Advertising topic: {topic} with type: {msg_type}")

    def publish_message(self, topic, msg):
        # ROS2에 메시지 게시
        if self.is_connected:
            message = {
                "op": "publish",
                "topic": topic,
                "msg": msg
            }
            self.ws.send(json.dumps(message))
            print(f"Published message: {msg}")
        else:
            print("WebSocket is not connected")

# 서버에 신호를 보내는 함수
async def send_signal(signal):
    uri = "ws://192.168.0.131:8765"  # WebSocket 서버 주소

    async with websockets.connect(uri) as websocket:
        await websocket.send(signal)
        response = await websocket.recv()
        print(f"서버 응답: {response}")

# 서버로 0, 1 값을 전송 후 ROS2 메시지 전송
async def send_values_to_server():
    """QR 코드 처리 후 서버로 0, 1 값을 전송"""
    await send_signal('0')  # 서버에 0 전송
    await asyncio.sleep(5)   # 5초 대기
    await send_signal('1')  # 5초 후 서버에 1 전송

    # 서버로 '1'을 전송한 후 ROS2에 메시지 보내기
    ros_client = ROS2WebSocketClient()
    ros_client.connect()  # ROS2 서버에 연결
    # 연결 후 ROS2 메시지 전송
    ros_client.publish_message("/robot_arm_command", {"data": "start"})  # ROS2에 "start" 메시지 보내기

def process_frame(frame):
    """QR 코드 인식을 위한 프레임 처리"""
    global qr_recognition_enabled

    # QR 코드 인식이 활성화되어 있지 않으면 프레임을 그대로 반환
    if not qr_recognition_enabled:
        return frame

    qr_codes = decode(frame)

    for qr in qr_codes:
        qr_data = qr.data.decode('utf-8')
        print(f"QR 코드 데이터: {qr_data}")

        # QR 코드가 등록된 주소 목록에 있는지 확인
        if qr_data in valid_addresses:
            print("등록된 주소 인식됨. 't' 전송.")
            arduino.write(b't')  # 아두이노로 't' 전송

        else:
            print("등록되지 않은 주소 인식됨. 'f' 전송.")
            arduino.write(b'f')  # 아두이노로 'f' 전송

        # QR 코드 인식 후 0 값을 서버로 전송하고 5초 후에 1 값을 서버로 전송
        asyncio.run(send_values_to_server())

        # QR 코드 인식 후 10초 동안 인식을 멈추도록 설정
        qr_recognition_enabled = False
        # 10초 후에 인식 활성화
        time.sleep(10)
        qr_recognition_enabled = True

    return frame

def main():
    cap = cv2.VideoCapture(1)  # 웹캠 열기

    while True:
        ret, frame = cap.read()  # 프레임 읽기
        if not ret:
            break

        # QR 코드 처리
        frame = process_frame(frame)

        # 화면을 띄우지 않고 QR 코드 인식 처리만 수행
        # cv2.imshow("QR Code Scanner", frame)  # 화면 띄우지 않음

        # 'q' 키로 종료 (이 부분은 화면을 띄우지 않더라도 유지)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
