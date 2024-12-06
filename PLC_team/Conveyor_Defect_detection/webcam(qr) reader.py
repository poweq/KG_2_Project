import cv2
from pyzbar.pyzbar import decode
import numpy as np  # numpy 임포트 추가

# 웹캠 열기
cap = cv2.VideoCapture(1)  # 0번이 기본 웹캠, 다른 번호로 설정 가능

while True:
    # 웹캠에서 이미지 캡처
    ret, frame = cap.read()

    if not ret:
        print("웹캠에서 이미지를 가져오지 못했습니다.")
        break

    # QR 코드 디코딩
    decoded_objects = decode(frame)

    for obj in decoded_objects:
        # QR 코드의 정보를 텍스트로 출력
        qr_data = obj.data.decode('utf-8')
        print(f"QR 코드 데이터: {qr_data}")

        # QR 코드의 테두리를 화면에 그리기
        points = obj.polygon
        if len(points) == 4:
            pts = [tuple(point) for point in points]
            cv2.polylines(frame, [np.array(pts, dtype=np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)

        # QR 코드 데이터 텍스트를 이미지에 출력
        cv2.putText(frame, qr_data, (obj.rect[0], obj.rect[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    # 결과 이미지 출력
    cv2.imshow("QR Code Reader", frame)

    # 'q' 키를 눌러 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 웹캠 해제 및 윈도우 닫기
cap.release()
cv2.destroyAllWindows()
