import cv2
import asyncio
import serial
from pyzbar.pyzbar import decode
import json
import websockets
import websocket
from threading import Thread

# 아두이노 연결 설정
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

# 등록된 QR 코드 주소 리스트
valid_addresses = [
    "https://site.naver.com/patient/A_1",
    "https://site.naver.com/patient/A_2",
    "https://site.naver.com/patient/A_3",
    "https://site.naver.com/patient/B_1",
    "https://site.naver.com/patient/B_2",
    "https://site.naver.com/patient/B_3",
]

class ROS2WebSocketClient:
    def __init__(self, rosbridge_ip='192.168.0.200', rosbridge_port=9090):
        self.rosbridge_ip = rosbridge_ip
        self.rosbridge_port = rosbridge_port
        self.ws = None
        self.is_connected = False

    def connect(self):
        def run():
            self.ws.run_forever()

        websocket.enableTrace(False)
        ws_url = f"ws://{self.rosbridge_ip}:{self.rosbridge_port}"
        self.ws = websocket.WebSocketApp(
            ws_url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        self.thread = Thread(target=run)
        self.thread.daemon = True
        self.thread.start()

    def on_open(self, ws):
        self.is_connected = True
        print(f"Connected to ROS2 at {self.rosbridge_ip}:{self.rosbridge_port}")
        self.advertise_topic("/robot_arm_command", "std_msgs/String")

    def on_message(self, ws, message):
        print(f"Received message: {message}")

    def on_error(self, ws, error):
        print(f"Error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        self.is_connected = False
        print(f"Connection closed: {close_msg}")

    def advertise_topic(self, topic, msg_type):
        try:
            advertise_message = {
                "op": "advertise",
                "topic": topic,
                "type": msg_type
            }
            self.ws.send(json.dumps(advertise_message))
            print(f"Advertising topic: {topic} with type: {msg_type}")
        except Exception as e:
            print(f"Failed to advertise topic: {e}")

    def publish_message(self, topic, msg):
        if not self.is_connected:
            print("WebSocket is not connected. Cannot publish message.")
            return
        try:
            message = {
                "op": "publish",
                "topic": topic,
                "msg": msg
            }
            self.ws.send(json.dumps(message))
            print(f"Published message: {msg}")
        except Exception as e:
            print(f"Failed to publish message: {e}")

async def send_signal(signal):
    uri = "ws://192.168.0.131:8765"  # PLC 서버
    async with websockets.connect(uri) as websocket:
        await websocket.send(signal)
        response = await websocket.recv()
        print(f"서버 응답: {response}")

async def process_frame_async(frame, cap):
    qr_codes = decode(frame)

    if qr_codes:  # QR 코드가 감지된 경우에만 처리
        # 카메라 일시 중지
        cap.release()
        print("QR 코드 감지됨. 카메라를 일시 중지합니다.")

        for qr in qr_codes:
            qr_data = qr.data.decode('utf-8')
            print(f"QR 코드 데이터: {qr_data}")

            # 웹 서버로 0 보내기
            await send_signal('0')

            if qr_data in valid_addresses:
                print("등록된 주소 인식됨. 't' 전송.")
                arduino.write(b't')
                await asyncio.sleep(2)
                # 웹 서버로 1 보내기
                await send_signal('1')
                await asyncio.sleep(2)

                # ROS2로 메시지 보내기
                ros_client = ROS2WebSocketClient()
                ros_client.connect()
                await asyncio.sleep(5)  # 연결 대기 시간 추가
                if ros_client.is_connected:
                    ros_client.publish_message("/robot_arm_command", {"data": "start"})
                else:
                    print("ROS2 연결 실패로 메시지를 보낼 수 없습니다.")
            else:
                print("등록되지 않은 주소 인식됨. 'f' 전송.")
                arduino.write(b'f')
                await asyncio.sleep(2)
                # 웹 서버로 1 보내기
                await send_signal('1')

        # 카메라 다시 활성화
        cap.open(0)
        print("QR 코드 처리가 완료되었습니다. 카메라를 다시 활성화합니다.")
        return True  # QR 코드가 감지된 경우에만 True 반환

    # QR 코드가 감지되지 않았을 경우
    arduino.write(b'f')  # 아두이노로 'f' 값 보내기
    print("QR 코드가 감지되지 않았습니다. 'f' 전송.")
    return False  # QR 코드가 감지되지 않으면 False 반환

async def main_async():
    await send_signal('1')
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("웹캠에서 프레임을 가져올 수 없습니다.")
            break

        # QR 코드 처리
        qr_detected = await process_frame_async(frame, cap)

        if qr_detected:
            print("QR 코드가 처리되었습니다.")
        else:
            print("QR 코드가 감지되지 않았습니다.")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(main_async())
