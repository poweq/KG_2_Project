from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO

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

# 하나의 창에 두 결과를 표시하는 함수
def show_combined_view():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # YOLO 객체 인식 결과 생성
        results = model(frame)
        frame_with_yolo = results[0].plot()  # 객체 인식 결과 표시용 복사본 생성

        # 중심점 표시용 프레임 복사
        frame_with_center = frame.copy()
        for result in results:
            for box in result.boxes:
                if box.conf >= CONFIDENCE_THRESHOLD:
                    # 바운딩 박스 좌표 추출
                    x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                    x_center = (x_min + x_max) // 2
                    y_center = (y_min + y_max) // 2

                    # 바운딩 박스 중심에 빨간 점 표시
                    cv2.circle(frame_with_center, (x_center, y_center), 10, (0, 0, 255), -1)

        # YOLO 객체 인식과 중심점 표시를 하나의 창에 겹쳐서 합침
        combined_frame = cv2.addWeighted(frame_with_yolo, 0.5, frame_with_center, 0.5, 0)

        # 병합된 프레임 출력
        cv2.imshow("Combined View", combined_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q'를 누르면 종료
            break

    cap.release()
    cv2.destroyAllWindows()

# 병합된 화면을 표시하는 함수 호출
show_combined_view()

# MyCobot 동작 - 포즈 설정 및 제어
mc.send_angles([3, -3, -1, -71, 90, 2], 20)  # pose2 위치로 이동
time.sleep(10)  # pose2에서 10초 대기

# 작업 완료 후 pose5로 돌아가기
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
print("모든 작업을 완료하고 pose5로 복귀합니다.")
