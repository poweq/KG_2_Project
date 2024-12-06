import cv2
import numpy as np
import websockets
import asyncio
import serial
from pyzbar.pyzbar import decode
import time

# 아두이노 연결 설정
arduino = serial.Serial('COM3', 9600, timeout=1)

# 등록된 QR 코드 주소 리스트
valid_addresses = [
    "https://site.naver.com/patient/B_3",
    "https://example.com/address1",
    "https://example.com/address2"
    # 추가적으로 더 많은 주소를 여기에 넣을 수 있습니다.
]

# QR 코드 인식 여부를 제어하는 변수
qr_recognition_enabled = True

async def send_signal(signal):
    uri = "ws://172.30.1.31:8765"  # WebSocket 서버 주소
    
    async with websockets.connect(uri) as websocket:
        await websocket.send(signal)
        response = await websocket.recv()
        print(f"서버 응답: {response}")

async def send_values_to_server():
    """QR 코드 처리 후 서버로 0, 1 값을 전송"""
    await send_signal('0')  # 서버에 0 전송
    await asyncio.sleep(5)   # 5초 대기
    await send_signal('1')  # 5초 후 서버에 1 전송

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
            # 서버로 't' 전송은 이제 하지 않음

        else:
            print("등록되지 않은 주소 인식됨. 'f' 전송.")
            arduino.write(b'f')  # 아두이노로 'f' 전송
            # 서버로 'f' 전송은 이제 하지 않음

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
        
        # 프레임 화면에 표시
        cv2.imshow("QR Code Scanner", frame)
        
        # 'q' 키로 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
