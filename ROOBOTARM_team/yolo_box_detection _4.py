from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO
import threading

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# MyCobot 도구 좌표 초기 설정
mc.set_tool_reference([245.6, -64.8, -40.2, 178.07, -0.15, -83.39])
current_x, current_y = 245.6, -64.8  # 초기 x, y 좌표

# YOLO 모델 로드
model = YOLO('C:\\Users\\shims\\Desktop\\github\\KG_2_Project\\ROOBOTARM_team\\yolov8_detect_model\\runs\\detect\\train2\\weights\\best.pt')

# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)

# 웹캠 설정
cap = cv2.VideoCapture(1)  # 한 개의 웹캠 사용
CONFIDENCE_THRESHOLD = 0.7  # 신뢰도 임계값 설정
TARGET_X, TARGET_Y = 300, 300  # 화면 중심 위치 설정
MOVE_SPEED = 10  # 로봇 이동 속도 조정

# 창 이름 정의
window_name = "YOLO Detection View"

# 바운딩 박스 중심점에 맞춰 로봇 팔을 조정하는 함수
def adjust_robot_position(x_center, y_center):
    global current_x, current_y

    # 중심 위치 오차 계산
    error_x = TARGET_X - x_center
    error_y = TARGET_Y - y_center
    
    # x, y 오차에 따른 좌표 조정 비율 설정
    adjust_x = error_x * 0.1  # x축 이동 비율 (필요에 따라 조정)
    adjust_y = -error_y * 0.1  # y축 이동 비율 (필요에 따라 조정, y 방향은 반대로 설정)

    # 새 좌표 계산
    current_x += adjust_x
    current_y += adjust_y

    # 새로운 좌표로 MyCobot 이동
    print(f"Moving to Coordinates: x={current_x}, y={current_y}")  # 이동할 좌표 디버깅 출력
    mc.set_tool_reference([current_x, current_y, -40.2, 178.07, -0.15, -83.39])
    time.sleep(0.1)  # 이동 후 잠시 대기

# YOLO 및 중심점을 표시하는 함수
def show_yolo_view_with_center():
    start_time = time.time()
    while time.time() - start_time < 20:  # 20초 동안 빨간 점 인식
        ret, frame = cap.read()
        if not ret:
            break

        # YOLO 모델 적용
        results = model(frame)
        frame_with_yolo = results[0].plot()  # YOLO Detection 화면

        # 바운딩 박스 중심점에 빨간 점 표시 및 로봇 제어 함수 호출
        for result in results:
            for box in result.boxes:
                if box.conf >= CONFIDENCE_THRESHOLD:
                    # 바운딩 박스 좌표 추출
                    x_min, y_min, x_max, y_max = map(int, box.xyxy[0])

                    # 중심 계산
                    x_center = (x_min + x_max) // 2
                    y_center = (y_min + y_max) // 2

                    # 바운딩 박스 중심에 빨간 점 표시
                    cv2.circle(frame_with_yolo, (x_center, y_center), 5, (0, 0, 255), -1)

                    # 인식된 좌표 출력
                    print(f"Detected Red Dot Position: ({x_center}, {y_center})")

                    # 중심점을 화면 (300, 300)으로 맞추기 위해 로봇 제어
                    adjust_robot_position(x_center, y_center)

        # 창 크기 설정 (너비, 높이)
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 600, 600)

        # YOLO Detection View에 출력
        cv2.imshow(window_name, frame_with_yolo)

        if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q'를 누르면 종료
            break

    cap.release()
    cv2.destroyAllWindows()

# 스레드 실행
yolo_thread = threading.Thread(target=show_yolo_view_with_center, daemon=True)
yolo_thread.start()

# MyCobot 동작 - pose2 위치로 이동
mc.send_angles([6, -13, -51, -26, 90, 2], 20)  # pose2 위치로 이동
time.sleep(10)  # pose2에서 10초 대기

# 작업 완료 후 pose5로 돌아가기
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
print("모든 작업을 완료하고 pose5로 복귀합니다.")
