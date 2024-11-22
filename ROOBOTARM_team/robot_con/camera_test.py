import cv2

# 각 장치 인덱스를 순차적으로 테스트
for i in range(4):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Device /dev/video{i} opened successfully.")
        ret, frame = cap.read()
        if ret:
            cv2.imshow(f'Webcam /dev/video{i}', frame)
            cv2.waitKey(0)  # 아무 키나 누르면 창 닫힘
            cv2.destroyAllWindows()
        cap.release()
    else:
        print(f"Could not open device /dev/video{i}")
