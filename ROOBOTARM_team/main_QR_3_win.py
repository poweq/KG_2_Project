from pymycobot.mycobot import MyCobot
import time
import cv2
from ultralytics import YOLO

# MyCobot 연결 설정
try:
    mc = MyCobot('COM6', 115200)
except Exception as e:
    print(f"로봇 연결 실패: {e}")
    exit()

# YOLO 모델 로드
try:
    model = YOLO('C:\\Users\\shims\\Desktop\\github\\KG_2_Project\\ROOBOTARM_team\\yolov8_model\\runs\\detect\\train2\\weights\\best.pt')
except Exception as e:
    print(f"YOLO 모델 로드 실패: {e}")
    exit()

# 픽셀-로봇 좌표 변환 비율 설정
PIXEL_TO_ROBOT_X = 0.2
PIXEL_TO_ROBOT_Y = 0.2

# 고정된 pose2 좌표 및 Z축 설정
POSE2_COORDS = [30.9, -327.5, 261.9, -170.94, 0.15, 170]
FIXED_Z = POSE2_COORDS[2]
LOWERED_Z = FIXED_Z - 100

# QR 코드 데이터 저장
last_detected_qr = None

# 카메라 초기화 함수
def init_camera():
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("카메라 초기화 실패")
        return None
    return cap

# 카메라 해제 함수
def release_camera(cap):
    if cap:
        cap.release()
    cv2.destroyAllWindows()

# QR 코드 인식
def detect_qr_code(cap):
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        return None

    detector = cv2.QRCodeDetector()
    data, points, _ = detector.detectAndDecode(frame)
    if points is not None and data:
        print(f"QR 코드 인식됨: {data}")
        return data.split("/patient/")[-1] if "/patient/" in data else "알 수 없음"
    return None

# 로봇 위치 이동 함수
def move_to_position(x, y, z, rx, ry, rz, speed=20):
    print(f"Moving to position: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    mc.send_coords([x, y, z, rx, ry, rz], speed)

# QR 코드 데이터 기반 블록 배치
def block_box_match():
    global last_detected_qr

    x, y = POSE2_COORDS[0], POSE2_COORDS[1]
    z = mc.get_coords()[2]  # 현재 Z축 위치 가져오기
    rx, ry, rz = POSE2_COORDS[3], POSE2_COORDS[4], POSE2_COORDS[5]

    if last_detected_qr == 'A_1':
        x += 100
        y += 50
        print("A_1 블록: 왼쪽 아래로 이동합니다.")
    elif last_detected_qr == 'A_2':
        y += 50
        print("A_2 블록: 중앙 아래로 이동합니다.")
    elif last_detected_qr == 'A_3':
        x -= 100
        y += 50
        print("A_3 블록: 오른쪽 아래로 이동합니다.")
    elif last_detected_qr == 'B_1':
        x += 100
        y -= 50
        print("B_1 블록: 왼쪽 위로 이동합니다.")
    elif last_detected_qr == 'B_2':
        y -= 50
        print("B_2 블록: 중앙 위로 이동합니다.")
    elif last_detected_qr == 'B_3':
        x -= 100
        y -= 50
        print("B_3 블록: 오른쪽 위로 이동합니다.")
    else:
        print("유효하지 않은 QR 코드입니다. 기본 위치로 이동합니다.")

    print(f"블록 배치 위치: x={x}, y={y}, z={z}, rx={rx}, ry={rz}")
    move_to_position(x, y, z, rx, ry, rz)

# QR 코드 감지 및 블록 잡기
def detect_and_grab_block(cap):
    global last_detected_qr

    # pose0로 이동
    mc.send_angles([-15, 60, 17, 5, -90, -14], 20)
    time.sleep(5)

    detected_qr = detect_qr_code(cap)
    if detected_qr:
        last_detected_qr = detected_qr
        print(f"탐지된 QR 코드: {last_detected_qr}")

        # 블록 잡기
        mc.send_angles([-13, 83, -2, -6, -90, -14], 20)
        time.sleep(3)
        mc.set_gripper_mode(0)
        mc.init_gripper()
        mc.set_gripper_state(1, 20, 1)
        time.sleep(3)
        print("블록을 성공적으로 잡았습니다!")
        return True
    else:
        print("QR 코드를 감지하지 못했습니다.")
        return False

# 메인 로직
def main():
    global last_detected_qr

    cap = init_camera()
    if not cap:
        return

    if detect_and_grab_block(cap):
        # pose1로 이동
        mc.send_angles([-15, 30, 11, 0, -90, 0], 20)
        time.sleep(5)

        # pose2로 이동 후 Z축 내리기
        move_to_position(POSE2_COORDS[0], POSE2_COORDS[1], FIXED_Z, POSE2_COORDS[3], POSE2_COORDS[4], POSE2_COORDS[5])
        time.sleep(5)
        move_to_position(POSE2_COORDS[0], POSE2_COORDS[1], LOWERED_Z, POSE2_COORDS[3], POSE2_COORDS[4], POSE2_COORDS[5])
        time.sleep(5)

        # QR 코드 데이터 기반 블록 배치
        block_box_match()
        time.sleep(5)

        # 그리퍼 열기
        mc.set_gripper_state(0, 20, 1)
        time.sleep(3)
        print("그리퍼 열기 완료")

        # 초기 위치로 이동
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)
        time.sleep(5)
        print("작업 완료 및 초기 위치로 복귀")
    else:
        # 초기 위치로 복귀
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)
        time.sleep(5)
        print("QR 코드 감지 실패로 복귀")

    release_camera(cap)

if __name__ == "__main__":
    for i in range(6):
        print(f"작업 {i + 1} 시작")
        main()
        print(f"작업 {i + 1} 완료")
