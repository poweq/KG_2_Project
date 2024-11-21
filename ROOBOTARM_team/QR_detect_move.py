from pymycobot.mycobot import MyCobot
import time
import cv2

# MyCobot 연결 설정
mc = MyCobot('/dev/ttyACM0', 115200)

# 웹캠 설정
cap = cv2.VideoCapture(2)
WINDOW_NAME = "QR Code Detection"

# QR 코드 인식 함수
def detect_qr_code(frame):
    detector = cv2.QRCodeDetector()
    data, points, _ = detector.detectAndDecode(frame)
    if points is not None and data:
        print(f"QR 코드 인식됨! 내용: {data}")
        return data
    return None

# QR 코드 내용에 따른 각도로 이동
def move_to_position_based_on_qr(qr_data):
    # QR 코드 내용에 따라 목표 각도 설정
    angles = {
        "A": [0, 20, 30, 0, 0, 0],  # 예제 위치 A
        "B": [20, -20, 30, 0, 0, 0], # 예제 위치 B
        "C": [10, 15, 30, 0, 0, 0]   # 예제 위치 C
    }

    if qr_data in angles:
        target_angles = angles[qr_data]
        print(f"QR 코드 '{qr_data}'에 해당하는 각도로 이동: {target_angles}")
        mc.send_angles(target_angles, 20)
        time.sleep(5)  # 이동 시간 대기
        print(f"{qr_data} 위치로 이동 완료!")
    else:
        print(f"QR 코드 내용 '{qr_data}'에 해당하는 각도가 없습니다.")

# QR 코드 인식 및 처리
def detect_and_process_qr_code():
    while True:
        ret, frame = cap.read()
        if not ret:
            print("카메라에서 프레임을 가져올 수 없습니다.")
            break

        # QR 코드 인식
        qr_data = detect_qr_code(frame)
        if qr_data:
            # QR 코드 내용에 따라 각도 이동
            move_to_position_based_on_qr(qr_data)
            break

        # 실시간으로 QR 코드 감지 업데이트
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 600, 600)
        cv2.imshow(WINDOW_NAME, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# 메인 함수
if __name__ == "__main__":
    # 초기 위치로 이동
    print("초기 위치로 이동 중...")
    mc.send_angles([-15, 65, 16, 0, -90, 0], 20)
    time.sleep(5)  # 초기 위치로 이동 후 대기

    # 초기 위치에서 QR 코드 확인
    print("QR 코드 확인 중...")
    detect_and_process_qr_code()
