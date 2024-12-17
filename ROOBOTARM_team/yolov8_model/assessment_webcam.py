import cv2
from ultralytics import YOLO

# YOLO 모델 로드
model = YOLO('C:\\Users\\shims\\Desktop\\github\\KG_2_Project\\ROOBOTARM_team\\yolov8_model\\runs\\detect\\train2\\weights\\best.pt')

# 웹캠 캡처 객체 생성 (1은 사용 중인 웹캠 인덱스)
cap = cv2.VideoCapture(1)

# 창 이름 지정
WINDOW_NAME = "YOLO Webcam"

# 창 생성 및 크기 조정 허용
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, 600, 600)

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        break

    # YOLO 모델 적용
    results = model(frame)

    # 결과 플로팅
    plots = results[0].plot()

    # 플롯된 이미지 표시
    cv2.imshow(WINDOW_NAME, plots)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 웹캠 릴리스 및 창 닫기
cap.release()
cv2.destroyAllWindows()
