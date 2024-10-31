from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)

# 색상별 위치 설정
def move_to_position(color):
    if color == 'red':
        # 빨간색 블록 위치
        mc.send_angles([60, -50, -21, 22, 79, 68], 20)
        time.sleep(3)
        mc.send_angles([-16, -16, -53, -13, 90, -17], 20)
        time.sleep(3)
        mc.send_angles([-16, -86, -4, -2, 90, -23], 20)
        time.sleep(3)
    elif color == 'blue':
        # 파란색 블록 위치
        mc.send_angles([60, -50, -21, 22, 79, 68], 20)
        time.sleep(3)
        mc.send_angles([29, -20, -37, -29, 90, 29], 20)
        time.sleep(3)
        mc.send_angles([23, -65, -38, 14, 90, 29], 20)
        time.sleep(3)
    elif color == 'yellow':
        # 노란색 블록 위치
        mc.send_angles([60, -50, -21, 22, 79, 68], 20)
        time.sleep(3)
        mc.send_angles([13, -10, -48, -27, 90, 10], 20)
        time.sleep(3)
        mc.send_angles([6, -60, -50, 22, 90, 10], 20)
        time.sleep(3)
    elif color == 'green':
        # 초록색 블록 위치
        mc.send_angles([60, -50, -21, 22, 79, 68], 20)
        time.sleep(3)
        mc.send_angles([-80, -20, -30, 20, 90, 20], 20)
        time.sleep(3)
        mc.send_angles([0, 50, -30, 20, 10, 10], 20)
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
    mc.send_angles([64, -82, -18, 24, 75, 68], 20)
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
