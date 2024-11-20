import cv2
import serial
from pyzbar.pyzbar import decode
import time

# 시리얼 포트 설정
ser = serial.Serial('COM4', 9600)  # COM 포트는 실제 사용 중인 포트로 변경
time.sleep(2)  # 시리얼 연결이 안정될 때까지 대기

# 웹캠으로 QR 코드 스캔
cap = cv2.VideoCapture(0)

# QR 코드 인식 상태 플래그와 마지막 전송 시간 초기화
qr_detected = False  # 현재 QR 코드가 인식되었는지 여부
last_sent_time = time.time()  # 마지막 전송 시간

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # QR 코드 인식
    decoded_objects = decode(frame)

    # 현재 시간
    current_time = time.time()

    if decoded_objects:
        # QR 코드가 인식된 상태로 처음 바뀔 때만 'a' 전송
        if not qr_detected and current_time - last_sent_time >= 2:
            print("QR Code detected!")
            ser.write(b'a')
            qr_detected = True  # QR 코드 인식 상태 업데이트
            last_sent_time = current_time  # 전송 후 시간 업데이트
    else:
        # QR 코드가 인식되지 않은 상태로 처음 바뀔 때만 'b' 전송
        if qr_detected and current_time - last_sent_time >= 2:
            print("QR Code lost!")
            ser.write(b'b')
            qr_detected = False  # QR 코드 인식 상태 업데이트
            last_sent_time = current_time  # 전송 후 시간 업데이트

    # QR 코드 화면에 표시
    cv2.imshow("QR Code Scanner", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
