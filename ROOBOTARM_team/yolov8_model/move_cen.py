import cv2
from ultralytics import YOLO

# YOLO 모델 로드
model = YOLO('C:\\Users\\shims\\Desktop\\github\\yolo_dectect_traing\\runs\\detect\\train\\weights\\best.pt')

# 웹캡처 객체 생성
cap = cv2.VideoCapture(1)

# 로봇 암 초기화 (예시로 함수 호출)
initialize_robot_arm()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # YOLO 모델 적용
    results = model(frame)

    found_red_space = False
    for result in results[0].boxes:
        if result.cls == 'red_space':  # 'red_space' 클래스 인식
            found_red_space = True
            x1, y1, x2, y2 = map(int, result.xyxy[0])  # 좌표 추출
            center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2  # 중심 좌표 계산
            
            # 로봇 암을 중심 좌표로 이동
            move_robot_arm_to(center_x, center_y)

            # 중심 좌표 표시
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

    cv2.imshow("YOLO Webcam", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

def initialize_robot_arm():
    # 로봇 암 초기화 코드
    pass

def move_robot_arm_to(x, y):
    # 로봇 암을 (x, y) 좌표로 이동하는 코드
    # 예: 로봇의 좌표계에 맞게 변환하고 제어 명령을 전송
    # 필요에 따라 Z축 이동도 고려해야 함
    print(f"Moving robot arm to: ({x}, {y})")
