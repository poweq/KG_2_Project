import cv2
import numpy as np


# 웹캠 설정
cap = cv2.VideoCapture(1)

# HSV 블록 색상 확인 함수
def detect_color():
    # HSV 색상 범위 설정
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([130, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    lower_red_1 = np.array([0, 100, 100])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([170, 100, 100])
    upper_red_2 = np.array([180, 255, 255])

    # 카메라로부터 색상 탐지
    while True:
        ret, frame = cap.read()
        if not ret:
            print("카메라에서 영상을 읽을 수 없습니다.")
            break

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        red_mask1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
        red_mask2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
        red_mask = cv2.add(red_mask1, red_mask2)

        # 색상 감지 시 해당 색상 출력
        if cv2.countNonZero(blue_mask) > 0:
            print("파란색 블록 탐지!")
        elif cv2.countNonZero(yellow_mask) > 0:
            print("노란색 블록 탐지!")
        elif cv2.countNonZero(green_mask) > 0:
            print("초록색 블록 탐지!")
        elif cv2.countNonZero(red_mask) > 0:
            print("빨간색 블록 탐지!")

        # 종료 조건 설정: 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("프로그램 종료.")
            break

    cap.release()
    cv2.destroyAllWindows()

# 메인 함수
def main():    
    # 지속적으로 색상 감지
    detect_color()

main()
