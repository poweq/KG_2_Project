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

# 웹캠 설정
cap = cv2.VideoCapture(1)  # 한 개의 웹캠 사용
CONFIDENCE_THRESHOLD = 0.7  # 신뢰도 임계값 설정

# YOLO 및 중심점을 표시하는 함수
def show_yolo_view_with_center():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # YOLO 모델 적용
        results = model(frame)
        frame_with_yolo = results[0].plot()  # YOLO Detection 화면

        # 바운딩 박스 중심점에 빨간 점 표시
        for result in results:
            for box in result.boxes:
                if box.conf >= CONFIDENCE_THRESHOLD:
                    # 바운딩 박스 좌표 추출
                    x_min, y_min, x_max, y_max = map(int, box.xyxy[0])

                    # 중심 계산
                    x_center = (x_min + x_max) // 2
                    y_center = (y_min + y_max) // 2

                    # 바운딩 박스 중심에 빨간 점 표시
                    cv2.circle(frame_with_yolo, (x_center, y_center), 10, (0, 0, 255), -1)

        # YOLO Detection View에 출력
        cv2.imshow("YOLO Detection View", frame_with_yolo)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q'를 누르면 종료
            break

    cap.release()
    cv2.destroyAllWindows()

# 스레드 실행
yolo_thread = threading.Thread(target=show_yolo_view_with_center, daemon=True)
yolo_thread.start()

# MyCobot 동작 - 포즈 설정 및 제어
mc.send_angles([3, -3, -1, -71, 90, 2], 20)  # pose2 위치로 이동
time.sleep(50)  # pose2에서 10초 대기

# 작업 완료 후 pose5로 돌아가기
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
print("모든 작업을 완료하고 pose5로 복귀합니다.")
