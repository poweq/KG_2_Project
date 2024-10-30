'''
from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# model = YOLO('C:\\Users\\shims\\Desktop\\github\\yolo_dectect_traing\\runs\\detect\\train\\weights\\best.pt')
# 웹캠 캡처 객체 생성 (0은 기본 웹캠을 의미)
# cap = cv2.VideoCapture(1)

# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)   


#컨베이어 밸트에서 내려온 블록 잡는 위치
#pose0
mc.send_angles([60, -90, 0, 10, 85, 60], 20)
time.sleep(3)
#pose1
mc.send_angles([60, -50, 0, 10, 85, 60], 20)
time.sleep(3)

#처음 블록을 잡고 파랑 -> 노랑 -> 빨강의 박스를 지나가면서 확인
#잡고 있는 블록의 색깔과 일치하면 블록을 놔
#파란색 박스 확인 위치
#pose2
mc.send_angles([29, -20, -37, -29, 90, 29], 20)
time.sleep(3)
# 파란색 박스에 블록 놓는 위치
# mc.send_angles([29, -70, -34, -1, 90, 29], 20)
# time.sleep(3)

#pose3
# 노란색 박스 확인 위치
mc.send_angles([13, -10, -48, -27, 90, 10], 20)
time.sleep(3)
# 노란색 박스에 블록 놓는 위치
# mc.send_angles([6, -60, -50, 22, 90, 10], 20)
# time.sleep(3)

#pose4
# 빨간색 박스 확인 위치
mc.send_angles([-16, -16, -53, -13, 90, -17], 20)
time.sleep(3)
# 빨간색 박스에 블록 놓는 위치
# mc.send_angles([-16, -86, -4, -2, 90, -23], 20)
# time.sleep(3)

# 초기 위치로 복귀
#pose5
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
time.sleep(3)

# 색상별 위치 설정
def move_to_position(color):
    if color == 'red':
        # 빨간색 블록 위치
        mc.send_angles([60, -50, 0, 10, 85, 60], 20)
        time.sleep(3)
        mc.send_angles([0, -50, 0, -40, 85, 0], 20)
        time.sleep(3)
        mc.send_angles([-5, -80, -17, 10, 85, 0], 20)
        time.sleep(3)
    elif color == 'blue':
        # 파란색 블록 위치
        mc.send_angles([60, -50, 0, 10, 85, 60], 20)
        time.sleep(3)
        mc.send_angles([38, -50, 0, -14, 90, 40], 20)
        time.sleep(3)
        mc.send_angles([38, -80, 0, -14, 90, 40], 20)
        time.sleep(3)
    elif color == 'yellow':
        # 노란색 블록 위치
        mc.send_angles([60, -50, 0, 10, 85, 60], 20)
        time.sleep(3)
        mc.send_angles([17, -50, 0, -10, 90, 20], 20)
        time.sleep(3)
        mc.send_angles([17, -80, 0, -10, 90, 20], 20)
        time.sleep(3)
    elif color == 'green':
        # 초록색 블록 위치
        mc.send_angles([60, -50, 0, 10, 85, 60], 20)
        time.sleep(3)
        mc.send_angles([-80, -20, -30, 20, 90, 20], 20)
        time.sleep(3)
        mc.send_angles([-80, -20, -90, 20, 90, 20], 20)
        time.sleep(3)

# 색상 탐지 함수
def detect_color():
    # 카메라 초기화
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return None

    # 파란색, 노란색, 초록색, 빨간색의 HSV 범위 설정
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

    # 탐지되지 않은 시간 초기화
    start_time = time.time()

    while True:
        # 프레임 읽기
        ret, frame = cap.read()
        if not ret:
            print("프레임을 수신할 수 없습니다. 스트림이 종료되었습니다.")
            break

        # BGR 이미지를 HSV로 변환
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 색상별 마스크 생성
        blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        red_mask1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
        red_mask2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
        red_mask = cv2.add(red_mask1, red_mask2)

        # 윤곽선을 찾아 색상 인식
        contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours_blue:
            print("파란색 탐지!")
            cap.release()
            cv2.destroyAllWindows()
            return 'blue'
        elif contours_yellow:
            print("노란색 탐지!")
            cap.release()
            cv2.destroyAllWindows()
            return 'yellow'
        elif contours_green:
            print("초록색 탐지!")
            cap.release()
            cv2.destroyAllWindows()
            return 'green'
        elif contours_red:
            print("빨간색 탐지!")
            cap.release()
            cv2.destroyAllWindows()
            return 'red'

        # 10초 동안 색상이 탐지되지 않으면 종료
        if time.time() - start_time > 10:
            print("10초 동안 색상이 탐지되지 않았습니다. 초기 위치로 돌아갑니다.")
            cap.release()
            cv2.destroyAllWindows()
            return None

        # 결과 이미지 표시 (디버깅용)
        cv2.imshow('Frame', frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return None

# 작업 수행 함수
def perform_action():
    # 초기 위치로 이동 후 색상 탐지
    mc.send_angles([60, -90, 0, 10, 85, 60], 20)
    time.sleep(3)

    # 색상 탐지
    color = detect_color()

    if color:
        # 색상에 맞는 위치로 이동
        print(f"{color} 블록을 잡습니다.")
        
        # 그리퍼 닫기 (블록 잡기)
        mc.set_gripper_state(1, 20, 1)
        time.sleep(3)
        
        # 블록을 색깔 위치로 옮기기
        move_to_position(color)

        # 그리퍼 열기 (블록 놓기)
        print(f"{color} 블록 놓기!")
        mc.set_gripper_state(0, 20, 1)
        time.sleep(3)

        # 초기 위치로 복귀
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)
        time.sleep(3)

    else:
        # 색상이 탐지되지 않으면 초기 위치로 복귀
        print("색상 인식 실패! 초기 위치로 돌아가며 그리퍼를 엽니다.")
        mc.set_gripper_state(0, 20, 1)  # 그리퍼 열기
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)
        time.sleep(3)

# 4번 반복
for i in range(4):
    perform_action()
    print(f"{i+1}번째 작업 완료")
'''




'''
from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)

# 색상 탐지 함수
def detect_color():
    # 카메라 초기화
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return None

    # 파란색, 노란색, 초록색, 빨간색의 HSV 범위 설정
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

    # 탐지되지 않은 시간 초기화
    start_time = time.time()

    while True:
        # 프레임 읽기
        ret, frame = cap.read()
        if not ret:
            print("프레임을 수신할 수 없습니다. 스트림이 종료되었습니다.")
            break

        # BGR 이미지를 HSV로 변환
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 색상별 마스크 생성
        blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        red_mask1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
        red_mask2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
        red_mask = cv2.add(red_mask1, red_mask2)

        # 윤곽선을 찾아 색상 인식
        contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours_blue:
            print("파란색 탐지!")
            cap.release()
            cv2.destroyAllWindows()
            return 'blue'
        elif contours_yellow:
            print("노란색 탐지!")
            cap.release()
            cv2.destroyAllWindows()
            return 'yellow'
        elif contours_green:
            print("초록색 탐지!")
            cap.release()
            cv2.destroyAllWindows()
            return 'green'
        elif contours_red:
            print("빨간색 탐지!")
            cap.release()
            cv2.destroyAllWindows()
            return 'red'

        # 10초 동안 색상이 탐지되지 않으면 종료
        if time.time() - start_time > 10:
            print("10초 동안 색상이 탐지되지 않았습니다. 초기 위치로 돌아갑니다.")
            cap.release()
            cv2.destroyAllWindows()
            return None

        # 결과 이미지 표시 (디버깅용)
        cv2.imshow('Frame', frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return None

# pose0: 컨베이어 벨트에서 내려온 블록을 확인하는 위치로 이동 후 색상 감지
mc.send_angles([60, -90, 0, 10, 85, 60], 20)
time.sleep(3)

# 색상 확인
detected_color = detect_color()
if detected_color:
    print(f"탐지된 색상: {detected_color}")
    # 블록을 감지했으므로 그리퍼로 잡기
    mc.set_gripper_state(0, 50)  # 그리퍼 닫기
    time.sleep(2)
    print("블록을 성공적으로 잡았습니다!")

    # pose1부터 pose5까지 이동
    # pose1
    mc.send_angles([60, -50, 0, 10, 85, 60], 20)
    time.sleep(3)
    
    # pose2
    mc.send_angles([29, -20, -37, -29, 90, 29], 20)
    time.sleep(3)
    
    # pose3
    mc.send_angles([13, -10, -48, -27, 90, 10], 20)
    time.sleep(3)
    
    # pose4
    mc.send_angles([-16, -16, -53, -13, 90, -17], 20)
    time.sleep(3)
    
    # pose5
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    time.sleep(3)
    
    print("모든 위치를 순서대로 이동 완료했습니다.")
else:
    print("블록의 색상을 감지하지 못했습니다.")
'''









from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# YOLO 모델 로드
model = YOLO('C:\\Users\\shims\\Desktop\\github\\yolo_dectect_traing\\runs\\detect\\train\\weights\\best.pt')

# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)

# HSV 색상 탐지 함수 (웹캠 화면 표시 추가)
def detect_color():
    cap = cv2.VideoCapture(1)  # 웹캠 인덱스, 필요에 따라 0으로 변경 가능
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return None

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

        # 웹캠 화면 표시
        cv2.imshow("Robot View", frame)  # 로봇이 보는 화면을 실시간으로 보여줌

        if cv2.countNonZero(blue_mask) > 0:
            print("파란색 탐지!")
            time.sleep(3)
            cap.release()
            cv2.destroyAllWindows()
            return 'blue'
        elif cv2.countNonZero(yellow_mask) > 0:
            print("노란색 탐지!")
            time.sleep(3)
            cap.release()
            cv2.destroyAllWindows()
            return 'yellow'
        elif cv2.countNonZero(green_mask) > 0:
            print("초록색 탐지!")
            time.sleep(3)
            cap.release()
            cv2.destroyAllWindows()
            return 'green'
        elif cv2.countNonZero(red_mask) > 0:
            print("빨간색 탐지!")
            time.sleep(3)
            cap.release()
            cv2.destroyAllWindows()
            return 'red'

        # 10초 동안 감지되지 않으면 종료
        if time.time() - start_time > 10:
            print("10초 동안 색상이 감지되지 않았습니다.")
            cap.release()
            cv2.destroyAllWindows()
            return None

        # 'q' 키를 누르면 화면을 종료할 수 있게 설정
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return None

# pose0에서 블록 색상 확인
mc.send_angles([71, -81, 0, 7, 83, 72], 20)
time.sleep(7)

# 블록 색상 탐지
detected_color = detect_color()
if detected_color:
    print(f"탐지된 색상: {detected_color}")
    mc.set_gripper_state(1 , 20, 1)  # 블록을 잡기 위해 그리퍼 닫기
    time.sleep(2)
    print("블록을 성공적으로 잡았습니다!")

    # pose1부터 pose5까지 이동
    poses = [
        [60, -50, 0, 10, 85, 60],   # pose1
        [29, -20, -37, -29, 90, 29], # pose2
        [13, -10, -48, -27, 90, 10], # pose3
        [-16, -16, -53, -13, 90, -17], # pose4
        [0, 0, 0, 0, 0, 0]           # pose5
    ]

    for i, pose in enumerate(poses, start=1):
        print(f"pose{i} 위치로 이동")
        mc.send_angles(pose, 20)
        time.sleep(3)  # 각 위치에서 대기 시간 조정
else:
    print("블록의 색상을 감지하지 못했습니다.")
