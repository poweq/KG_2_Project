import cv2

def main():
    camera_index = 1  # /dev/video2에 해당
    cap = cv2.VideoCapture(camera_index)

    # 웹캠이 제대로 열렸는지 확인
    if not cap.isOpened():
        print(f"Error: Cannot access webcam at /dev/video{camera_index}")
        return

    print("Press 'q' to quit the video stream.")
    
    while True:
        # 프레임 읽기
        ret, frame = cap.read()

        # 프레임이 제대로 읽히지 않았으면 종료
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # 프레임을 창에 표시
        cv2.imshow("Webcam Live Stream", frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 리소스 해제
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
