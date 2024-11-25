import cv2
import socket
import pickle
import struct

# UDP 설정
video_receive_port = 5005  # Raspberry Pi에서 영상 데이터를 수신할 포트 번호

# 소켓 생성 및 바인딩
video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_sock.bind(("192.168.0.182", video_receive_port))

print("Raspberry Pi로부터 영상 데이터 수신 중...")

try:
    while True:
        # 영상 데이터 수신
        try:
            video_data, video_addr = video_sock.recvfrom(65507)
            video_size = struct.unpack("Q", video_data[:8])[0]
            video_frame_data = video_data[8:8 + video_size]
            img = pickle.loads(video_frame_data)
            img = cv2.imdecode(img, cv2.IMREAD_COLOR)

            if img is None:
                print("프레임 디코딩 실패, 다음 프레임으로 넘어갑니다.")
                continue

            # 화면에 영상 출력
            cv2.imshow('Received Video', img)
            key = cv2.waitKey(1)
            if key & 0xff == ord('q'):  # 'q'를 눌러 종료
                break
        except Exception as e:
            print(f"영상 데이터 수신 중 오류 발생: {e}")
            continue

finally:
    cv2.destroyAllWindows()
    video_sock.close()
    print("프로그램이 정상적으로 종료되었습니다.")
