from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np

# MyCobot 객체 초기화 (시리얼 포트에 맞게 수정)
mc = MyCobot("COM6", 115200)

# pose 지점의 각도 설정
pose0_position = [71, -81, 0, 7, 83, 72]  # 블록 색상 인식 위치
pose1_position = [60, -50, 0, 10, 85, 60]
pose2_position = [3, -3, -1, -71, 90, 2]  # 박스 인식 위치

# 감지된 색상을 저장할 변수 선언
last_detected_color = None

# HSV 색상 탐지 함수
def detect_color():
    # 웹캠 초기화
    cap = cv2.VideoCapture(1) 
    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
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

        if cv2.countNonZero(blue_mask) > 0:
            print("파란색 블록 탐지!")
            detected_color = 'blue'
            break
        elif cv2.countNonZero(yellow_mask) > 0:
            print("노란색 블록 탐지!")
            detected_color = 'yellow'
            break
        elif cv2.countNonZero(green_mask) > 0:
            print("초록색 블록 탐지!")
            detected_color = 'green'
            break
        elif cv2.countNonZero(red_mask) > 0:
            print("빨간색 블록 탐지!")
            detected_color = 'red'
            break
        else:
            detected_color = None

        # 화면에 현재 프레임 표시
        cv2.imshow("Robot View", frame)

        # 'q'를 누르면 중단
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # 10초 동안 색상이 감지되지 않으면 None 반환
        if time.time() - start_time > 10:
            print("10초 동안 색상이 감지되지 않았습니다.")
            detected_color = None
            break

    cap.release()
    cv2.destroyAllWindows()
    return detected_color

# pose0 지점으로 이동
mc.send_angles(pose0_position, 20)
time.sleep(3)

# 블록 색상 탐지 및 기억
detected_color = detect_color()
if detected_color:
    last_detected_color = detected_color  # 블록 색상 기억
    print(f"탐지된 색상: {detected_color}")
    mc.set_gripper_state(1, 20, 1)  # 블록을 잡기 위해 그리퍼 닫기
    time.sleep(2)
    print("블록을 성공적으로 잡았습니다!")
else:
    print("블록의 색상을 감지하지 못했습니다.")

# pose1 지점으로 이동
mc.send_angles(pose1_position, 20)
time.sleep(3)

# pose2 지점으로 이동
mc.send_angles(pose2_position, 20)
time.sleep(3)

# 로봇 초기화
mc.send_angles([0, 0, 0, 0, 0, 0], 20)  # 초기 위치로 이동
time.sleep(2)
