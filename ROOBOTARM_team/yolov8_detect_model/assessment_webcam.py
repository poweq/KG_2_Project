import cv2
from ultralytics import YOLO

# YOLO 모델 로드
model = YOLO('C:\\Users\\shims\\Desktop\\github\\yolo_dectect_traing\\runs\\detect\\train\\weights\\best.pt')

# 웹캠 캡처 객체 생성 (0은 기본 웹캠을 의미)
cap = cv2.VideoCapture(1)

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
    cv2.imshow("YOLO Webcam", plots)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 웹캠 릴리스 및 창 닫기
cap.release()
cv2.destroyAllWindows()
