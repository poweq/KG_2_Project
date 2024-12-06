import websocket
import json

# WebSocket 메시지 수신 콜백 함수
def on_message(ws, message):
    data = json.loads(message)
    print(f"Received from ROS 2: {data}")

# WebSocket 연결 열림 시 초기화 메시지 전송
def on_open(ws):
    # ROS 2 토픽 구독 메시지
    subscribe_message = {
        "op": "subscribe",
        "topic": "/robot_arm_status"  # ROS 2에서 보내는 실제 토픽 이름을 사용
    }
    ws.send(json.dumps(subscribe_message))
    print("Subscribed to /robot_arm_status")

# WebSocket 오류 콜백
def on_error(ws, error):
    print(f"WebSocket error: {error}")

# WebSocket 연결 종료 콜백
def on_close(ws, close_status_code, close_msg):
    print("WebSocket connection closed")

# WebSocket 서버 URL (라즈베리파이의 IP 주소로 변경)
ROSBRIDGE_SERVER_URL = "ws://192.168.0.200:9090"  # 실제 라즈베리파이 IP로 변경하세요

# WebSocket 연결 설정
ws = websocket.WebSocketApp(
    ROSBRIDGE_SERVER_URL,
    on_message=on_message,
    on_open=on_open,
    on_error=on_error,
    on_close=on_close,
)

# WebSocket 실행
ws.run_forever()
