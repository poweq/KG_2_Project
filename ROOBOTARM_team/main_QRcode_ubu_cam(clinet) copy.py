import cv2
import socket
import struct
import pickle
import time

# UDP 설정
pc_ip = "192.168.0.162"  # Windows PC의 IP 주소
video_send_port = 5005  # 영상 데이터를 송신할 포트 번호

# 영상 송신을 위한 소켓 생성
video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 카메라 설정 (웹캠 사용)
cap = cv2.VideoCapture(0)  # 0번 장치(기본 웹캠) 사용
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # 카메라 해상도 너비 설정
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # 카메라 해상도 높이 설정

# 카메라가 정상적으로 열렸는지 확인
if not cap.isOpened():
    print("카메라를 열 수 없습니다. 프로그램을 종료합니다.")
    exit(1)

print("카메라 데이터 송신 중... (Ctrl + C로 종료)")

try:
    # 메인 송신 루프
    while True:
        # 카메라로부터 프레임 읽기
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다. 프로그램을 종료합니다.")
            break

        # 프레임을 JPEG로 인코딩
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ret:
            print("프레임 인코딩에 실패하였습니다.")
            continue

        # 인코딩된 프레임을 직렬화하여 UDP로 전송
        try:
            data = pickle.dumps(buffer)  # 직렬화
            video_sock.sendto(struct.pack("Q", len(data)) + data, (pc_ip, video_send_port))
            print("프레임을 성공적으로 전송하였습니다.")
        except Exception as e:
            print(f"영상 데이터 송신 중 오류 발생: {e}")
            continue

        # 프레임 송신 주기 조절 (FPS 설정)
        time.sleep(0.033)  # 약 30fps로 송신 (필요에 따라 조절 가능)

except KeyboardInterrupt:
    print("사용자에 의해 프로그램이 중단되었습니다.")

finally:
    # 리소스 정리
    cap.release()
    video_sock.close()
    print("프로그램이 정상적으로 종료되었습니다.")
