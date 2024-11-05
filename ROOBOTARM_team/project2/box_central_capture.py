import cv2
from ultralytics import YOLO

# YOLO 모델 로드
model = YOLO('C:\\Users\\shims\\Desktop\\github\\KG_2_Project\\ROOBOTARM_team\\yolov8_detect_model\\runs\\detect\\train2\\weights\\best.pt')

# 웹캠에서 객체 중심을 인식하고 좌표 반환하는 함수
def detect_object_center():
    # 웹캠 설정
    cap = cv2.VideoCapture(1)
    CONFIDENCE_THRESHOLD = 0.7
    TARGET_X, TARGET_Y = 300, 300
    WINDOW_NAME = "YOLO Detection View"

    # 전역 변수 선언
    global current_x, current_y, centered, first_detection
    current_x, current_y = 0, 0  # 초기화
    centered = False
    first_detection = True

    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        cap.release()
        return None, None, False

    # YOLO 모델 적용
    results = model(frame)
    frame_with_yolo = results[0].plot()

    # 빨간 점 인식 및 조정
    for result in results:
        for box in result.boxes:
            if box.conf >= CONFIDENCE_THRESHOLD:
                # 바운딩 박스의 중심 계산
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                x_center = (x_min + x_max) // 2
                y_center = (y_min + y_max) // 2

                # 중심점 위치에 빨간 점 표시
                cv2.circle(frame_with_yolo, (x_center, y_center), 5, (0, 0, 255), -1)

                # 중심점 위치 출력
                if first_detection:
                    print(f"중심점 위치: ({x_center}, {y_center})")
                    first_detection = False

                # 중심을 화면 중앙으로 맞추기 위해 x, y 값만 조정
                adjust_x = (TARGET_X - x_center) * 1
                adjust_y = (TARGET_Y - y_center) * 1

                # 새로운 x, y 값 계산 및 업데이트
                current_x += adjust_x
                current_y += adjust_y

                # 중심이 거의 중앙에 맞으면 centered 설정
                if abs(x_center - TARGET_X) < 10 and abs(y_center - TARGET_Y) < 10:
                    centered = True
                    print("중심이 화면 중앙에 맞춰졌습니다.")
                    
                # 화면에 결과 표시
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(WINDOW_NAME, 600, 600)
                cv2.imshow(WINDOW_NAME, frame_with_yolo)
                cv2.waitKey(0)  # 한 프레임 표시 후 대기
                
                # 웹캠 및 창 닫기
                cap.release()
                cv2.destroyAllWindows()
                
                # 결과 리턴
                return current_x, current_y, centered

    # 만약 객체를 인식하지 못하면 웹캠 닫기
    cap.release()
    cv2.destroyAllWindows()
    return None, None, False  # 객체가 없으면 None 반환

# 카메라 x축 차이 만큼 로봇 y축에 더하기

