#박스의 위치를 칸마다 인식해서 블록과 색깔 일치
from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO
import threading

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# YOLO 모델 로드
model = YOLO('C:\\Users\\shims\\Desktop\\github\\KG_2_Project\\ROOBOTARM_team\\yolov8_detect_model\\runs\\detect\\train2\\weights\\best.pt')
# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)

# 감지된 색상을 저장할 변수 선언
last_detected_color = None
cap = cv2.VideoCapture(1)  # 로봇암에 달린 웹캠 1, 컴퓨터의 웹캠 0
CONFIDENCE_THRESHOLD = 0.7  # 신뢰도 임계값 설정

# 웹캠을 계속 켜두기 위한 함수
def keep_camera_open():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # pose2부터 pose4 사이에서는 YOLO 인식 결과만 표시
        if running_yolo:
            results = model(frame)  # YOLO 모델을 프레임에 적용
            frame = results[0].plot()  # 인식된 객체를 플로팅하여 표시

        # 웹캠 화면 표시
        cv2.imshow("Robot View", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q'를 누르면 종료
            break

    cap.release()
    cv2.destroyAllWindows()

# 웹캠을 계속 켜두기 위해 스레드 시작
running_yolo = False  # YOLO 실행 여부를 제어할 변수
camera_thread = threading.Thread(target=keep_camera_open, daemon=True)
camera_thread.start()

# HSV 색상 탐지 함수
def detect_color():
    global cap
    # HSV 색상 범위 설정
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([130, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    lower_red_1 = np.array([0, 100, 100])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([170, 100, 100])
    upper_red_2 = np.array([180, 255, 255])

    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        red_mask1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
        red_mask2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
        red_mask = cv2.add(red_mask1, red_mask2)

        if cv2.countNonZero(blue_mask) > 0:
            print("파란색 탐지!")
            return 'blue'
        elif cv2.countNonZero(yellow_mask) > 0:
            print("노란색 탐지!")
            return 'yellow'
        elif cv2.countNonZero(green_mask) > 0:
            print("초록색 탐지!")
            return 'green'
        elif cv2.countNonZero(red_mask) > 0:
            print("빨간색 탐지!")
            return 'red'

        if time.time() - start_time > 10:
            print("10초 동안 색상이 감지되지 않았습니다.")
            return None

# pose0에서 블록 색상 확인
mc.send_angles([71, -81, 0, 7, 83, 72], 20)
time.sleep(3)

# 블록 색상 탐지
detected_color = detect_color()
if detected_color:
    last_detected_color = detected_color  # 블록 색상 기억
    print(f"탐지된 색상: {detected_color}")
    mc.set_gripper_state(1, 20, 1)  # 블록을 잡기 위해 그리퍼 닫기
    time.sleep(2)
    print("블록을 성공적으로 잡았습니다!")
else:
    print("블록의 색상을 감지하지 못했습니다.")

# pose1부터 pose4까지 이동
poses = [
    [60, -50, 0, 10, 85, 60],   # pose1
    [29, -20, -37, -29, 90, 29], # pose2에서 blue_space 중심 좌표 찾기
    [13, -10, -48, -27, 90, 10], # pose3에서 yellow_space 중심 좌표 찾기
    [-16, -16, -53, -13, 90, -17] # pose4에서 red_space 중심 좌표 찾기
]

for i, pose in enumerate(poses, start=1):
    print(f"pose{i} 위치로 이동")
    mc.send_angles(pose, 20)
    time.sleep(3)  # 각 위치에서 대기 시간 조정

    # pose2부터 pose4까지는 YOLO 모델로 객체 인식을 켜고 화면 표시
    running_yolo = 2 <= i <= 4

    if running_yolo:  # YOLO 모델 적용 시기에만 객체 인식
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            continue

        # YOLO 모델을 사용해 객체 인식
        results = model(frame)
        print(f"pose{i}에서 인식된 객체:")

        # pose별로 특정 색상의 객체 중점 계산 및 표시
        target_name = None
        if i == 2:
            target_name = 'blue_space'
        elif i == 3:
            target_name = 'yellow_space'
        elif i == 4:
            target_name = 'red_space'

        # 지정된 객체의 중점 찾기
        for result in results:
            if target_name in result.names and result.confidence >= CONFIDENCE_THRESHOLD:
                # 경계 상자 중심 계산
                x_center = int(result.box.x + result.box.width / 2)
                y_center = int(result.box.y + result.box.height / 2)
                print(f"{target_name} 중점 좌표: ({x_center}, {y_center})")

                # 화면에 중점에 파란색 점 표시
                cv2.circle(frame, (x_center, y_center), 5, (0, 0, 0), -1)
                cv2.imshow("Robot View", frame)
                cv2.waitKey(1)
                break  # 첫 번째로 찾은 일치하는 객체에 대해서만 실행

# 작업 완료 후 pose5로 돌아가기
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
print("모든 작업을 완료하고 pose5로 복귀합니다.")
