#yolo 부분에 웹캠 켜지는 코드
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
model = YOLO('/home/shim/github/KG_2_Project/ROOBOTARM_team/yolov8_model/runs/detect/train6/weights/best.pt')

# 글로벌 변수(전역 변수) 선언
running = False               # 로봇 작업 진행 상태
signal_received = None        # 신호 상태
task_thread = None            # 로봇 작업 스레드
should_exit = False           # 프로그램 종료 플래그 추가
last_detected_qr = None       # QR 코드 데이터 저장
cap = None                    # 카메라 객체

# UDP 설정
signal_port = 7000
signal_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
signal_sock.bind(("0.0.0.0", signal_port))
signal_sock.settimeout(1.0)

# 픽셀-로봇 좌표 변환 비율 설정
pixel_to_robot_x = 0.2
pixel_to_robot_y = 0.2

# pose2 위치 (z축 고정)
pose2_coords = [86.9, -219.2, 352.8, -169.02, 0.54, 177.87]
fixed_z = pose2_coords[2]

# 초기 변수 설정
current_x, current_y = pose2_coords[0], pose2_coords[1]
centered = False # 중심 맞추기 완료 여부 확인
first_detection = True  # 처음 중심점 위치 출력 여부 확인

CONFIDENCE_THRESHOLD = 0.7
TARGET_X, TARGET_Y = 300, 300
WINDOW_NAME = "YOLO Detection View"


# 카메라 초기화함수
def init_camera():
    global cap
    if cap is not None and cap.isOpened():
        print("카메라가 이미 초기화되어 있습니다.")
        return cap

    cap = cv2.VideoCapture(2)
    if cap.isOpened():
        print("카메라 초기화 성공.")
        return cap
    else:
        print("카메라 초기화 실패.")
        cap = None
        return None

def release_camera():
    global cap
    if cap is not None:
        cap.release()  # 카메라 리소스 해제
        cv2.destroyAllWindows()
        cap = None
        print("카메라가 닫혔습니다.")

# QR 코드 인식 함수
def detect_qr_code():
    global cap
    if cap is None or not cap.isOpened():
        print("카메라가 초기화되지 않았습니다. 다시 초기화 시도 중...")
        cap = init_camera()
        if cap is None:
            print("카메라 재초기화 실패.")
            return None

    for attempt in range(10):  # QR 코드 감지를 최대 10회 시도
        ret, frame = cap.read()
        if not ret:
            print("카메라에서 프레임을 가져올 수 없습니다. 다시 시도 중...")
            cap = init_camera()
            continue

        print("QR 코드 감지 시도 중...")
        detector = cv2.QRCodeDetector()

        # 밝기 및 대비 조정
        for alpha in [1.0, 1.2, 1.5]:
            for beta in [0, 30, 60]:
                adjusted_frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
                data, points, _ = detector.detectAndDecode(adjusted_frame)
                if points is not None and data:
                    print(f"QR 코드 인식 성공: {data}")
                    return data

        print("QR 코드 감지 실패, 재시도...")
    print("QR 코드 감지 실패. 모든 시도 종료.")
    return None

# pose0에서 QR 코드 인식 및 블록 잡기
def detect_and_grab_block():
    global last_detected_qr

    # pose0로 이동
    mc.send_angles([-20, 7.29, 81.03, 1.66, -90, 70], 20)  # pose0_1 웹캠으로 QR 코드 확인 위치
    #-15, 60, 17, 5, -90, -14
    time.sleep(5)

    for attempt in range(10):  # QR 코드 감지를 최대 10회 시도
        detected_qr = detect_qr_code()
        if detected_qr:
            last_detected_qr = detected_qr
            print(f"탐지된 QR 코드: {detected_qr}")

            # 블록 잡기
            mc.send_angles([-23, 3.29, 81.03, 1.66, -90, 70], 20)
            time.sleep(1)
            mc.send_angles([-26.54, 15.55, 64.68, 14.5, -93.6, 64.68], 20)  # pose0_2 그리퍼로 블록 잡기 전에 구조물에 안걸리게 이동
            time.sleep(1)
            mc.send_angles([-25.4, 21.53, 88.33, -14.94, -93.51, 65.83], 20)  # pose0_3 그리퍼로 블록 잡는 위치
            time.sleep(3)
            mc.set_gripper_mode(0)
            mc.init_gripper()
            mc.init_eletric_gripper()
            mc.set_gripper_value(15,20,1)  # 그리퍼로 블록 잡기
            time.sleep(3)
            mc.send_angles([-25.48, 34.8, 14.32, 48.42, -93.33, 65.65], 20)  # pose1 그리퍼로 블록 잡은 후 위로 올린 위치
            time.sleep(3)
            print("블록을 성공적으로 잡았습니다!")
            return True

        print(f"QR 코드 감지 실패, {attempt + 1}/3 시도 완료.")
        time.sleep(2)  # 재시도 전 대기

    print("QR 코드를 감지하지 못했습니다. 작업 실패.")
    return False

# pose2에서 객체 인식 후 조정
def perform_pose2_adjustments():
    global centered
    centered = False  # 조정 시작 시 초기화
    mc.send_angles([90, 17.05, 17.75, 42.46, -90, 2], 20)  # pose2 위치로 이동
    time.sleep(5)
    
    # 중심 맞추기 시작 시간 기록
    start_time = time.time()
    time_limit = 10  # 10초 제한

    print("객체 중심 맞추기를 시작합니다.")

    while not centered and (time.time() - start_time) < time_limit:
        detect_and_adjust_position()

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

# 객체 감지 및 위치 조정 함수
def detect_and_adjust_position():
    global current_x, current_y, centered, first_detection
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        return

    # YOLO 모델 적용
    results = model(frame)
    frame_with_yolo = results[0].plot()

    # 보정 비율 설정 (픽셀 -> 로봇 좌표 변환)
    pixel_to_robot_x = 0.5  # 픽셀 이동에 따른 x축 보정 비율
    pixel_to_robot_y = 0.5  # 픽셀 이동에 따른 y축 보정 비율

    # 빨간 점 인식 후 조정
    for result in results:
        for box in result.boxes:
            if box.conf >= CONFIDENCE_THRESHOLD:
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
                adjust_x = (TARGET_X - x_center) * pixel_to_robot_x * (1)  # X축 부호 반전
                adjust_y = (TARGET_Y - y_center) * pixel_to_robot_y * (-1)  # Y축 부호 반전
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
        cap.release()
        cv2.destroyAllWindows()
        exit()

def lower_z():
    global current_x, current_y, lowered_z, lowered_y
    lowered_z = fixed_z - 150
    # lowered_y = current_y - 0
    lowered_y = current_y - 30

    print("로봇암을 아래로 내립니다.")
    move_to_position(current_x, lowered_y, lowered_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])
    time.sleep(5)

def block_box_match():
    x, y = current_x, lowered_y
    z = mc.get_coords()[2]  # 현재 z축 위치 가져오기
    rx, ry, rz = pose2_coords[3], pose2_coords[4], pose2_coords[5]

    print(f"디버깅: 감지된 QR 코드 = {last_detected_qr}")

    # QR 코드 데이터에 따라 블록 배치 위치 설정
    if last_detected_qr == 'https://site.naver.com/patient/A_1':
        x += 60
        y -= 80
        print("A_1 블록: 왼쪽 위로 이동합니다.")
    elif last_detected_qr == 'https://site.naver.com/patient/A_2':
        x += 60
        print("A_2 블록: 왼쪽으로 이동합니다.")
    elif last_detected_qr == 'https://site.naver.com/patient/A_3':
        x += 60
        y += 70
        print("A_3 블록: 왼쪽 아래로 이동합니다.")
    elif last_detected_qr == 'https://site.naver.com/patient/B_1':
        x -= 60
        y -= 80
        print("B_1 블록: 오른쪽 위로 이동합니다.")
    elif last_detected_qr == 'https://site.naver.com/patient/B_2':
        x -= 60
        print("B_2 블록: 중앙 위로 이동합니다.")        
    elif last_detected_qr == 'https://site.naver.com/patient/B_3':
        x -= 60
        y += 65
        print("B_3 블록: 오른쪽 아래로 이동합니다.")
        
    print(f"블록을 놓는 위치로 이동: x={x}, y={y}, z={z}, rx={rx}, ry={rz}")
    move_to_position(x, y, z, rx, ry, rz)
    mc.set_gripper_value(100,20,1) #그리퍼 열기
    print("그리퍼 열기...")
    time.sleep(5)

def reset_robot():
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    mc.set_gripper_value(100,20,1)  # 그리퍼 열기
    time.sleep(7)
    print("로봇이 초기 위치로 돌아갔습니다.")

###################################################################################################
# 로봇 작업 함수 (별도의 스레드에서 실행)
def robot_task():
    global running, last_detected_qr, should_exit
    try:
        # 작업 시작
        if not running or should_exit:
            return  # 실행 중단
####################################### 1 단계 #######################################
        if detect_and_grab_block():
            if not running or should_exit:
                return  # 실행 중단
####################################### 2단계 #######################################
            perform_pose2_adjustments()  # 10초 동안 감지 후 다음 단계로 진행
            print("객체 중심 맞추기...")
            if not running or should_exit:
                return  # 실행 중단
####################################### 3단계 #######################################
            lower_z()
            print("Z축 내리기...")
            if not running or should_exit:
                return  # 실행 중단
####################################### 4단계 #######################################
            block_box_match()
            print("블록 배치...")
            if not running or should_exit:
                return  # 실행 중단
####################################### 5단계 #######################################
            reset_robot()
            time.sleep(3)
        else:
            print("QR 코드 감지 실패 또는 블록 잡기 실패. 작업을 종료합니다.")
            reset_robot()
    finally:
        running = False  # 작업 종료 표시

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

###################################################################################################

# 메인 루프
def main():
    global cap, should_exit, signal_received
    signal_thread = None
    process_signal_thread = None
    try:
        # MyCobot 연결 확인
        if not check_robot_connection():
            print("MyCobot 연결 실패. 프로그램을 종료합니다.")
            return

        print("MyCobot 초기화 중...")
        reset_robot()
        time.sleep(2)
        print("초기화 완료.")

        # 카메라 초기화
        cap = init_camera()
        if cap is None:  # 초기화 실패 시 종료
            print("카메라 초기화 실패. 프로그램을 종료합니다.")
            return

        # 신호 수신 스레드 시작
        signal_thread = threading.Thread(target=listen_for_signal, daemon=True)
        signal_thread.start()

        # 신호 처리 루프 시작
        process_signal_thread = threading.Thread(target=process_signal, daemon=True)
        process_signal_thread.start()

        # 메인 루프: 신호 처리 및 프로그램 종료 대기,
        print("서버 신호를 기다리는 중입니다.")
        while not should_exit:
            if signal_received:  # 새로운 신호가 수신된 경우
                if signal_received.lower() == 'q':  # 'q' 신호로 종료
                    print("서버로부터 종료 신호(q)를 수신했습니다.")
                    should_exit = True
                else:
                    print(f"수신된 신호 처리 중: {signal_received}")
                signal_received = None  # 신호 초기화
            time.sleep(1)  # CPU 과부하 방지를 위한 대기

    finally:
        # 카메라 및 리소스 해제
        if cap:
            release_camera(cap)
            cap = None
        print("프로그램 종료.")

        # 모든 스레드 종료 대기
        if signal_thread:
            signal_thread.join()
        if process_signal_thread:
            process_signal_thread.join()
        if task_thread and task_thread.is_alive():
            task_thread.join()

if __name__ == "__main__":
    main()