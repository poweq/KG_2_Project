from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO

# MyCobot 연결 설정
mc = MyCobot('/dev/ttyACM0', 115200)

# YOLO 모델 로드
model = YOLO('/home/shim/github/KG_2_Project/ROOBOTARM_team/yolov8_model/runs/detect/train2/weights/best.pt')

# 웹캠 설정 (전역 변수)
cap = cv2.VideoCapture(2)  # 전역 변수로 선언
WINDOW_NAME = "QR Code Detection"

# pose2 위치 설정
pose2_coords = [30.9, -327.5, 261.9, -170.94, 0.15, 170]
fixed_z = pose2_coords[2]  # z축 고정
lowered_z = fixed_z - 100  # z축 낮춘 위치

current_x, current_y = pose2_coords[0], pose2_coords[1]
last_detected_qr = None  # 전역 변수로 QR 코드 기억

# QR 코드 인식 함수
def detect_qr_code(frame):
    detector = cv2.QRCodeDetector()
    data, points, _ = detector.detectAndDecode(frame)
    if points is not None and data:
        print(f"QR 코드 인식됨! 내용: {data}")
        return data
    return None

# 특정 좌표로 이동
def move_to_position(x, y, z, rx, ry, rz, speed=20):
    print(f"Moving to position: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    mc.send_coords([x, y, z, rx, ry, rz], speed)
    time.sleep(5)

# pose2로 이동하는 함수
def move_to_pose2():
    print("pose2로 이동 중...")
    x, y, z, rx, ry, rz = pose2_coords
    move_to_position(x, y, z, rx, ry, rz, speed=20)
    print("pose2 도달 완료!")

# pose0에서 QR 코드 인식 후 블록 잡기
def detect_and_grab_block():
    global last_detected_qr
    global cap  # 전역 변수 cap 사용
    time.sleep(3)
    mc.send_angles([-15, 65, 16, 0, -90, 0], 20)  # pose0로 이동
    time.sleep(5)

    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        return False

    detected_qr = detect_qr_code(frame)
    if detected_qr:
        last_detected_qr = detected_qr  # QR 코드 데이터 저장
        print(f"탐지된 QR 코드: {detected_qr}")

        # 블록 잡기
        mc.send_angles([-15, 74, 11, 0, -90, 0], 20)  # 그리퍼 위치
        time.sleep(3)
        mc.set_gripper_mode(0)
        mc.init_gripper()
        mc.set_gripper_state(1, 20, 1)  # 그리퍼 닫기
        time.sleep(3)
        print("블록을 성공적으로 잡았습니다!")
        return True
    else:
        print("QR 코드를 감지하지 못했습니다.")
        return False

# QR 코드에 따라 블록 놓기
def block_box_match():
    x, y = current_x, current_y
    z = mc.get_coords()[2]  # 현재 z축 위치 가져오기
    rx, ry, rz = pose2_coords[3], pose2_coords[4], pose2_coords[5]

    if last_detected_qr == 'A':
        y += 120
        print("A 블록: 왼쪽으로 이동합니다.")
    elif last_detected_qr == 'B':
        print("B 블록: 중앙에 놓습니다.")
    elif last_detected_qr == 'C':
        y -= 120
        print("C 블록: 오른쪽으로 이동합니다.")

    print(f"블록을 놓는 위치로 이동: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    move_to_position(x, y, z, rx, ry, rz)

def lower_z():
    global lowered_z
    print("로봇암을 아래로 내립니다.")
    move_to_position(current_x, current_y, lowered_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])

def main():
    qr_detected = detect_and_grab_block()

    if qr_detected:
        move_to_pose2()  # pose2로 이동
        lower_z()        # Z축 내리기
        block_box_match() # QR 코드에 따라 위치 이동

        print("그리퍼 열기...")
        mc.set_gripper_state(0, 20, 1)  # 그리퍼 열기
        time.sleep(3)

        print("초기 위치로 복귀 중...")
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)  # 초기 위치
        time.sleep(5)
    else:
        print("QR 코드가 감지되지 않아 작업을 건너뜁니다.")

# 실행 루프
for i in range(4):
    main()
    print(f"{i+1}번째 작업 완료")

# 작업 완료 후 카메라 릴리스
cap.release()
