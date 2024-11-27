from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO
import socket
import threading

# MyCobot 연결 설정
mc = MyCobot('/dev/ttyACM0', 115200)

# 로봇 연결 확인 함수
def check_robot_connection():
    try:
        mc.get_angles()
        print("MyCobot 연결 성공")
        return True
    except Exception as e:
        print(f"MyCobot 연결 실패: {e}")
        return False

# YOLO 모델 로드
model = YOLO('/home/shim/github/KG_2_Project/ROOBOTARM_team/yolov8_model/runs/detect/train2/weights/best.pt')

# 글로벌 변수(전역 변수) 선언
running = False            # 로봇 작업 진행 상태
signal_received = None # 신호 상태
task_thread = None  # 로봇 작업 스레드
should_exit = False        # 프로그램 종료 플래그 추가

# UDP 설정
signal_port = 7000
signal_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
signal_sock.bind(("0.0.0.0", signal_port))
signal_sock.settimeout(1.0)

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

# 카메라 초기화 및 해제 함수
def init_camera():
    cap = cv2.VideoCapture(0)
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
    global cap
    if cap is None:
        print("카메라가 초기화되지 않았습니다.")
        return None

    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        return None

    print("QR 코드 감지 시도 중...")

    # 밝기 및 대비 조정
    for alpha in [1.0, 1.2, 1.5]:
        for beta in [0, 30, 60]:
            adjusted_frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
            detector = cv2.QRCodeDetector()
            data, points, _ = detector.detectAndDecode(adjusted_frame)
            if points is not None and data:
                print(f"QR 코드 인식 성공: {data}")
                return data

    print("QR 코드 감지 실패")
    return None

# pose0에서 QR 코드 인식 및 블록 잡기
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


###########################################################################
# 비동기적으로 신호를 수신하는 함수
def listen_for_signal():
    global signal_received, should_exit
    while not should_exit:
        try:
            print("신호 대기 중...")
            signal, addr = signal_sock.recvfrom(1024)
            signal = signal.decode().strip()
            print(f"수신된 신호: {signal}")
            signal_received = signal  # 전역 변수에 신호 저장
        except socket.timeout:
            continue  # 타임아웃 발생 시 다시 대기
        except Exception as e:
            print(f"신호 수신 중 오류 발생: {e}")
            continue

# 로봇 작업 함수 (별도의 스레드에서 실행)
def robot_task():
    global running, last_detected_qr, should_exit
    try:
        # 작업 시작
        if not running or should_exit:
            return  # 실행 중단
        if detect_and_grab_block():
            if not running or should_exit:
                return  # 실행 중단
            print("객체 중심 맞추기...")
            perform_pose2_adjustments()
            if not running or should_exit:
                return  # 실행 중단
            print("Z축 내리기...")
            lower_z()
            if not running or should_exit:
                return  # 실행 중단
            print("블록 배치...")
            block_box_match()
            if not running or should_exit:
                return  # 실행 중단
            print("그리퍼 열기...")
            mc.set_gripper_state(0, 20, 1)
            time.sleep(3)
            reset_robot()
        else:
            print("QR 코드 감지 실패 또는 블록 잡기 실패. 작업을 종료합니다.")
            reset_robot()
    finally:
        running = False  # 작업 종료 표시

# 신호 처리 함수
def process_signal():
    global signal_received, running, task_thread, should_exit
    while not should_exit:
        if signal_received:
            signal = signal_received
            signal_received = None  # 신호 초기화

            if signal == "1":
                if not running:
                    print("작업을 시작합니다.")
                    running = True
                    # 로봇 작업 스레드 시작
                    task_thread = threading.Thread(target=robot_task)
                    task_thread.start()
                else:
                    print("이미 작업이 진행 중입니다.")
            elif signal == "0":
                if running:
                    print("작업을 중단합니다.")
                    mc.stop()  # 로봇 동작 중단
                    running = False
                else:
                    print("작업이 진행 중이 아닙니다.")
            elif signal.lower() == "r":
                print("로봇 초기화 중...")
                reset_robot()
            elif signal.lower() == "q":
                print("프로그램 종료")
                reset_robot()
                should_exit = True  # 프로그램 종료 플래그 설정
            else:
                print("잘못된 신호 수신.")
        time.sleep(0.1)  # CPU 사용량을 낮추기 위해 약간 대기

##########################################################################

# pose2에서 객체 인식 후 조정
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
    time.sleep(2)  # 명령 실행 시간을 줌
    coords = mc.get_coords()
    print(f"현재 로봇 위치: {coords}")

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

#    if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' 키를 누르면 종료
#        release_camera(cap)
#        exit()

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

def reset_robot():
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    time.sleep(5)       
    print("로봇이 초기 위치로 돌아갔습니다.")

# 메인 루프
def main():
    global cap, should_exit
    try:
        if not check_robot_connection():
            print("MyCobot 연결 실패. 프로그램을 종료합니다.")
            return

        print("MyCobot 초기화 중...")
        reset_robot()
        time.sleep(2)
        print("초기화 완료.")

        cap = init_camera()
        if not cap:
            print("카메라 초기화 실패")
            return

        # 신호 수신 스레드 시작
        signal_thread = threading.Thread(target=listen_for_signal, daemon=True)
        signal_thread.start()

        # 신호 처리 루프 시작
        process_signal_thread = threading.Thread(target=process_signal, daemon=True)
        process_signal_thread.start()

        # 메인 스레드는 다른 작업을 수행하거나 대기
        while not should_exit:
            time.sleep(1)  # 메인 스레드는 대기 상태를 유지

    finally:
        if cap:
            release_camera(cap)
            cap = None
        print("프로그램 종료.")
        # 프로그램 종료를 위해 모든 스레드가 종료될 때까지 대기
        signal_thread.join()
        process_signal_thread.join()
        if task_thread and task_thread.is_alive():
            task_thread.join()

if __name__ == "__main__":
    main()