from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO
import socket

# MyCobot 연결 설정
mc = MyCobot('/dev/ttyACM0', 115200)

# YOLO 모델 로드
model = YOLO('/home/shim/github/KG_2_Project/ROOBOTARM_team/yolov8_model/runs/detect/train2/weights/best.pt')

# UDP 설정
signal_port = 7000
signal_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
signal_sock.bind(("0.0.0.0", signal_port))
signal_sock.settimeout(5.0)

# 픽셀-로봇 좌표 변환 비율 설정
pixel_to_robot_x = 0.2
pixel_to_robot_y = 0.2

# pose2 위치 (z축 고정)
pose2_coords = [30.9, -327.5, 261.9, -170.94, 0.15, 170]
fixed_z = pose2_coords[2]
lowered_z = fixed_z - 100

# 초기 변수 설정
cap = None
current_x, current_y = pose2_coords[0], pose2_coords[1]
centered = False # 중심 맞추기 완료 여부 확인
first_detection = True  # 처음 중심점 위치 출력 여부 확인

CONFIDENCE_THRESHOLD = 0.7
TARGET_X, TARGET_Y = 300, 300
WINDOW_NAME = "YOLO Detection View"

# QR 코드 데이터 저장
last_detected_qr = None

# 카메라 초기화 및 해제
def init_camera():
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        return None
    return cap

def release_camera(cap):
    if cap:
        cap.release()
    cv2.destroyAllWindows()

# QR 코드 인식 함수
def detect_qr_code():
    if cap is None:
        print("카메라가 초기화되지 않았습니다.")
        return None

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

# pose0에서 QR 코드 인식
def detect_and_grab_block():
    global last_detected_qr

    # pose0로 이동
    mc.send_angles([-15, 60, 17, 5, -90, -14], 20)  # pose0_1 웹캠으로 QR 코드 확인 위치
    time.sleep(5)

    detected_qr = detect_qr_code()
    if detected_qr:
        last_detected_qr = detected_qr
        print(f"탐지된 QR 코드: {detected_qr}")

        # 블록 잡기
        mc.send_angles([-13, 83, -2, -6, -90, -14], 20)  # pose0_2 그리퍼로 블록 잡는 위치
        time.sleep(3)
        mc.set_gripper_mode(0)
        mc.init_gripper()
        mc.set_gripper_state(1, 20, 1)  # 그리퍼로 블록 잡기
        time.sleep(3)
        print("블록을 성공적으로 잡았습니다!")
        return True
    else:
        print("QR 코드를 감지하지 못했습니다.")
        return False

# pose2에서 객체 인식 후 중점(빨간 점)을 인식하고 조정
def perform_pose2_adjustments():
    global centered
    centered = False  # 조정 시작 시 centered 초기화
    mc.send_angles([90, 5, 0, 57, -90, 0], 20)  # pose2
    time.sleep(5)

    # 중심 맞추기 시작 시간 기록
    start_time = time.time()
    time_limit = 10  # 10초 제한

    # 빨간 점을 화면 중앙으로 맞출 때까지 조정 (최대 10초 동안)
    while not centered and (time.time() - start_time) < time_limit:
        detect_and_adjust_position()

    # 10초 경과 후 또는 중심 맞추기 완료 후 처리
    if not centered:
        print(f"{time_limit}초 내에 현재 위치에서 로봇 암을 내립니다.")
    else:
        print("빨간 점이 목표 좌표 근처에 위치했습니다.")

def move_to_position(x, y, z, rx, ry, rz, speed=20):
    print(f"Moving to position: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    mc.send_coords([x, y, z, rx, ry, rz], speed)

def detect_and_adjust_position():
    global current_x, current_y, centered, first_detection, cap
    if cap is None:
        print("카메라가 초기화되지 않았습니다.")
        return

    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        return

    # YOLO 모델 적용
    results = model(frame[..., ::-1])  # BGR → RGB 변환
    frame_with_yolo = results[0].plot()

    for result in results:
        for box in result.boxes:
            if box.conf.item() >= CONFIDENCE_THRESHOLD:
                # 바운딩 박스의 중심 계산
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                x_center = (x_min + x_max) // 2
                y_center = (y_min + y_max) // 2

                cv2.circle(frame_with_yolo, (x_center, y_center), 5, (0, 0, 255), -1)

                # 중심점 위치 출력
                if first_detection:
                    print(f"중심점 위치(X좌표 , Y좌표) : ({x_center}, {y_center})")
                    first_detection = False

                # 부호 반대로 적용하여 조정값 계산
                adjust_x = (TARGET_X - x_center) * pixel_to_robot_x
                adjust_y = (TARGET_Y - y_center) * pixel_to_robot_y * -1
                print(f"조정값(X좌표 , Y좌표) : ({adjust_x}, {adjust_y})")

                # 새로운 x, y 값을 계산하고 업데이트
                current_x = pose2_coords[0] + adjust_x
                current_y = pose2_coords[1] + adjust_y

                # 로봇을 조정된 좌표로 이동
                move_to_position(current_x, current_y, fixed_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])

                # 중심이 목표 좌표에 근접했는지 확인
                if abs(x_center - TARGET_X) < 10 and abs(y_center - TARGET_Y) < 10:
                    centered = True
                    print("빨간 점이 목표 좌표 근처에 위치했습니다.")
                    return

    # 실시간으로 YOLO Detection View 업데이트
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 600, 600)
    cv2.imshow(WINDOW_NAME, frame_with_yolo)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' 키를 누르면 종료
        release_camera(cap)
        exit()

def lower_z():
    global current_x, current_y, lowered_z
    print("로봇암을 아래로 내립니다.")
    move_to_position(current_x, current_y, lowered_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])
    time.sleep(5)

def block_box_match():
    x, y = current_x, current_y
    z = mc.get_coords()[2]  # 현재 z축 위치 가져오기
    rx, ry, rz = pose2_coords[3], pose2_coords[4], pose2_coords[5]

    # QR 코드 데이터에 따라 블록 배치 위치 설정
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
        
    print(f"블록을 놓는 위치로 이동: x={x}, y={y}, z={z}, rx={rx}, ry={rz}")
    move_to_position(x, y, z, rx, ry, rz)

# 신호 대기 및 처리
def wait_for_signal():
    try:
        print("신호 대기 중...")
        signal, addr = signal_sock.recvfrom(1024)
        signal = signal.decode().strip()
        print(f"수신된 신호: {signal}")
        return signal
    except socket.timeout:
        print("신호 수신 대기 타임아웃 발생")
        return None
    except Exception as e:
        print(f"신호 수신 중 오류 발생: {e}")
        return None

def process_signal(signal):
    if signal == "1":
        print("작업을 계속 진행합니다.")
        return True
    elif signal == "0":
        print("대기 상태로 전환합니다.")
        return False
    elif signal.lower() == "r":
        print("로봇 초기화 중...")
        reset_robot()
        return False
    elif signal.lower() == "q":
        print("프로그램 종료")
        exit()
    else:
        print("잘못된 신호 수신.")
        return False

def wait_and_process_signal():
    signal = wait_for_signal()
    if signal is not None:
        return process_signal(signal)
    else:
        print("신호를 받지 못했습니다.")
        return False

def reset_robot():
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    time.sleep(5)


# 메인 작업 흐름
def main():
    global cap
    try:
        if cap is None:
            cap = init_camera()
            if not cap:
                print("카메라 초기화 실패")
                return

        while True:
            # QR 코드 탐지 및 작업 진행
            print("QR 코드 탐지 중...")
            qr_detected = detect_qr_code()

            if qr_detected:
                print("QR 코드 탐지 성공. 작업 진행.")
                mc.send_angles([-15, 30, 11, 0, -90, 0], 20)
                time.sleep(5)
                if not wait_and_process_signal():
                    reset_robot()
                    continue  # 대기 상태로 전환

                print("객체 중심 맞추기...")
                perform_pose2_adjustments()
                if not wait_and_process_signal():
                    reset_robot()
                    continue
                lower_z()  # Z축 내리기
                print("Z축 내리기...")
                move_to_position(current_x, current_y, lowered_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])
                time.sleep(5)
                if not wait_and_process_signal():
                    reset_robot()
                    continue

                print("블록 배치...")
                block_box_match()
                if not wait_and_process_signal():
                    reset_robot()
                    continue

                print("그리퍼 열기...")
                mc.set_gripper_state(0, 20, 1)
                time.sleep(3)

                reset_robot()
            else:
                print("QR 코드 탐지 실패. 신호 대기 중...")
                if not wait_and_process_signal():
                    reset_robot()
                    continue  # 대기 상태로 전환

    finally:
        if cap:
            release_camera(cap)
            cap = None
        print("프로그램 종료.")


if __name__ == "__main__":
    main()
