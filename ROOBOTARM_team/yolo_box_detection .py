# #컨베이어 밸트에서 내려온 블록 잡는 위치
# #pose0
# mc.send_angles([60, -90, 0, 10, 85, 60], 20)
# time.sleep(3)
# #pose1
# mc.send_angles([60, -50, 0, 10, 85, 60], 20)
# time.sleep(3)

# #처음 블록을 잡고 파랑 -> 노랑 -> 빨강의 박스를 지나가면서 확인
# #잡고 있는 블록의 색깔과 일치하면 블록을 놔
# #파란색 박스 확인 위치
# #pose2
# mc.send_angles([29, -20, -37, -29, 90, 29], 20)
# time.sleep(3)
# # 파란색 박스에 블록 놓는 위치
# # mc.send_angles([29, -70, -34, -1, 90, 29], 20)
# # time.sleep(3)

# #pose3
# # 노란색 박스 확인 위치
# mc.send_angles([13, -10, -48, -27, 90, 10], 20)
# time.sleep(3)
# # 노란색 박스에 블록 놓는 위치
# # mc.send_angles([6, -60, -50, 22, 90, 10], 20)
# # time.sleep(3)

# #pose4
# # 빨간색 박스 확인 위치
# mc.send_angles([-16, -16, -53, -13, 90, -17], 20)
# time.sleep(3)
# # 빨간색 박스에 블록 놓는 위치
# # mc.send_angles([-16, -86, -4, -2, 90, -23], 20)
# # time.sleep(3)

# # 초기 위치로 복귀
# #pose5
# mc.send_angles([0, 0, 0, 0, 0, 0], 20)
# time.sleep(3)


# '''
# from pymycobot.mycobot import MyCobot
# import time
# import cv2
# import numpy as np
# from ultralytics import YOLO

# # MyCobot 연결 설정
# mc = MyCobot('COM6', 115200)

# # YOLO 모델 로드
# model = YOLO('C:\\Users\\shims\\Desktop\\github\\yolo_dectect_traing\\runs\\detect\\train\\weights\\best.pt')

# # 그리퍼 모드 설정 및 초기화
# mc.set_gripper_mode(0)
# mc.init_gripper()
# mc.set_gripper_calibration()
# time.sleep(3)

# # 감지된 색상을 저장할 변수
# last_detected_color = None

# # HSV 색상 탐지 함수
# def detect_color():
#     cap = cv2.VideoCapture(1)
#     if not cap.isOpened():
#         print("카메라를 열 수 없습니다.")
#         return None

#     # HSV 색상 범위 설정
#     lower_blue = np.array([100, 100, 100])
#     upper_blue = np.array([130, 255, 255])
#     lower_yellow = np.array([20, 100, 100])
#     upper_yellow = np.array([30, 255, 255])
#     lower_green = np.array([35, 100, 100])
#     upper_green = np.array([85, 255, 255])
#     lower_red_1 = np.array([0, 100, 100])
#     upper_red_1 = np.array([10, 255, 255])
#     lower_red_2 = np.array([170, 100, 100])
#     upper_red_2 = np.array([180, 255, 255])

#     start_time = time.time()

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break

#         hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#         blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
#         yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
#         green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
#         red_mask1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
#         red_mask2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
#         red_mask = cv2.add(red_mask1, red_mask2)

#         if cv2.countNonZero(blue_mask) > 0:
#             print("파란색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'blue'
#         elif cv2.countNonZero(yellow_mask) > 0:
#             print("노란색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'yellow'
#         elif cv2.countNonZero(green_mask) > 0:
#             print("초록색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'green'
#         elif cv2.countNonZero(red_mask) > 0:
#             print("빨간색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'red'

#         if time.time() - start_time > 10:
#             print("10초 동안 색상이 감지되지 않았습니다.")
#             cap.release()
#             cv2.destroyAllWindows()
#             return None

# # pose0에서 블록 색상 확인
# mc.send_angles([71, -81, 0, 7, 83, 72], 20)
# time.sleep(7)

# # 블록 색상 탐지
# detected_color = detect_color()
# if detected_color:
#     last_detected_color = detected_color
#     print(f"탐지된 색상: {detected_color}")
#     mc.set_gripper_state(1, 20, 1)
#     time.sleep(2)
#     print("블록을 성공적으로 잡았습니다!")

#     # pose1부터 pose4까지 이동
#     poses = [
#         [60, -50, 0, 10, 85, 60],   # pose1
#         [29, -20, -37, -29, 90, 29], # pose2
#         [13, -10, -48, -27, 90, 10], # pose3
#         [-16, -16, -53, -13, 90, -17] # pose4
#     ]

#     # 웹캠 캡처 객체 생성
#     cap = cv2.VideoCapture(1)

#     for i, pose in enumerate(poses, start=1):
#         print(f"pose{i} 위치로 이동")
#         mc.send_angles(pose, 20)
#         time.sleep(3)

#         if 2 <= i <= 4:  # pose2부터 pose4까지 YOLO 모델 적용
#             ret, frame = cap.read()
#             if not ret:
#                 print("프레임을 읽을 수 없습니다.")
#                 continue

#             # YOLO 모델을 사용해 객체 인식
#             results = model(frame)
#             print(f"pose{i}에서 인식된 객체:")

#             # 인식된 객체가 잡은 블록과 일치하는지 확인
#             for result in results:
#                 if last_detected_color == 'red' and 'red_space' in result.names:
#                     mc.send_angles([-16, -86, -4, -2, 90, -23], 20)
#                     print("빨간색 블록을 지정된 위치에 내려놓습니다.")
#                     break
#                 elif last_detected_color == 'blue' and 'blue_space' in result.names:
#                     mc.send_angles([29, -70, -34, -1, 90, 29], 20)
#                     print("파란색 블록을 지정된 위치에 내려놓습니다.")
#                     break
#                 elif last_detected_color == 'yellow' and 'yellow_space' in result.names:
#                     mc.send_angles([6, -60, -50, 22, 90, 10], 20)
#                     print("노란색 블록을 지정된 위치에 내려놓습니다.")
#                     break
#                 elif last_detected_color == 'green':
#                     mc.send_angles([0, 50, -30, 20, 10, 10], 20)
#                     print("초록색 블록을 지정된 위치에 내려놓습니다.")
#                     break

#     cap.release()
#     cv2.destroyAllWindows()

#     # 작업 완료 후 pose5로 돌아가기
#     mc.send_angles([0, 0, 0, 0, 0, 0], 20)
#     print("모든 작업을 완료하고 pose5로 복귀합니다.")
# else:
#     print("블록의 색상을 감지하지 못했습니다.")
# '''



# from pymycobot.mycobot import MyCobot
# import time
# import cv2
# import numpy as np
# from ultralytics import YOLO

# # MyCobot 연결 설정
# mc = MyCobot('COM6', 115200)

# # YOLO 모델 로드
# model = YOLO('C:\\Users\\shims\\Desktop\\github\\yolo_dectect_traing\\runs\\detect\\train\\weights\\best.pt')

# # 그리퍼 모드 설정 및 초기화
# mc.set_gripper_mode(0)
# mc.init_gripper()
# mc.set_gripper_calibration()
# time.sleep(3)

# # 감지된 색상을 저장할 변수 선언
# last_detected_color = None

# # HSV 색상 탐지 함수
# def detect_color():
#     cap = cv2.VideoCapture(1)
#     if not cap.isOpened():
#         print("카메라를 열 수 없습니다.")
#         return None

#     # HSV 색상 범위 설정
#     lower_blue = np.array([100, 100, 100])
#     upper_blue = np.array([130, 255, 255])
#     lower_yellow = np.array([20, 100, 100])
#     upper_yellow = np.array([30, 255, 255])
#     lower_green = np.array([35, 100, 100])
#     upper_green = np.array([85, 255, 255])
#     lower_red_1 = np.array([0, 100, 100])
#     upper_red_1 = np.array([10, 255, 255])
#     lower_red_2 = np.array([170, 100, 100])
#     upper_red_2 = np.array([180, 255, 255])

#     start_time = time.time()

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break

#         hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#         blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
#         yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
#         green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
#         red_mask1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
#         red_mask2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
#         red_mask = cv2.add(red_mask1, red_mask2)

#         # 웹캠 화면 표시
#         cv2.imshow("Robot View", frame)

#         if cv2.countNonZero(blue_mask) > 0:
#             print("파란색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'blue'
#         elif cv2.countNonZero(yellow_mask) > 0:
#             print("노란색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'yellow'
#         elif cv2.countNonZero(green_mask) > 0:
#             print("초록색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'green'
#         elif cv2.countNonZero(red_mask) > 0:
#             print("빨간색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'red'

#         if time.time() - start_time > 10:
#             print("10초 동안 색상이 감지되지 않았습니다.")
#             cap.release()
#             cv2.destroyAllWindows()
#             return None
        
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

# # pose0에서 블록 색상 확인
# mc.send_angles([71, -81, 0, 7, 83, 72], 20) 
# time.sleep(7)

# # 블록 색상 탐지
# detected_color = detect_color()
# if detected_color:
#     last_detected_color = detected_color  # 블록 색상 기억
#     print(f"탐지된 색상: {detected_color}")
#     mc.set_gripper_state(1, 20, 1)  # 블록을 잡기 위해 그리퍼 닫기
#     time.sleep(2)
#     print("블록을 성공적으로 잡았습니다!")
# else:
#     print("블록의 색상을 감지하지 못했습니다.")

# # pose1부터 pose4까지 이동
# poses = [
#     [60, -50, 0, 10, 85, 60],   # pose1
#     [29, -20, -37, -29, 90, 29], # pose2 yolo로 위치 인식 시작
#     [13, -10, -48, -27, 90, 10], # pose3 
#     [-16, -16, -53, -13, 90, -17] # pose4 yolo로 위치 인식 끝
# ]

# cap = cv2.VideoCapture(1)  # 웹캠 캡처 객체 생성

# for i, pose in enumerate(poses, start=1):
#     print(f"pose{i} 위치로 이동")
#     mc.send_angles(pose, 20)
#     time.sleep(3)  # 각 위치에서 대기 시간 조정

#     if 2 <= i <= 4:  # pose2부터 pose4까지 YOLO 모델 적용
#         ret, frame = cap.read()
#         if not ret:
#             print("프레임을 읽을 수 없습니다.")
#             continue

#         # YOLO 모델을 사용해 객체 인식
#         results = model(frame)
#         print(f"pose{i}에서 인식된 객체:")

#         # 인식된 객체가 잡은 블록과 일치하는지 확인
#         for result in results:
#             if last_detected_color == 'red' and 'red_space' in result.names:
#                 mc.send_angles([-16, -86, -4, -2, 90, -23], 20)
#                 print("빨간색 블록을 지정된 위치에 내려놓습니다.")
#                 break
#             elif last_detected_color == 'blue' and 'blue_space' in result.names:
#                 mc.send_angles([29, -70, -34, -1, 90, 29], 20)
#                 print("파란색 블록을 지정된 위치에 내려놓습니다.")
#                 break
#             elif last_detected_color == 'yellow' and 'yellow_space' in result.names:
#                 mc.send_angles([6, -60, -50, 22, 90, 10], 20)
#                 print("노란색 블록을 지정된 위치에 내려놓습니다.")
#                 break
#             elif last_detected_color == 'green':
#                 mc.send_angles([0, 50, -30, 20, 10, 10], 20)
#                 print("초록색 블록을 지정된 위치에 내려놓습니다.")
#                 break

# cap.release()
# cv2.destroyAllWindows()

# # 작업 완료 후 pose5로 돌아가기
# mc.send_angles([0, 0, 0, 0, 0, 0], 20)
# print("모든 작업을 완료하고 pose5로 복귀합니다.")

# from pymycobot.mycobot import MyCobot
# import time
# import cv2
# import numpy as np
# from ultralytics import YOLO

# # MyCobot 연결 설정
# mc = MyCobot('COM6', 115200)

# # YOLO 모델 로드
# model = YOLO('C:\\Users\\shims\\Desktop\\github\\yolo_dectect_traing\\runs\\detect\\train\\weights\\best.pt')

# # 그리퍼 모드 설정 및 초기화
# mc.set_gripper_mode(0)
# mc.init_gripper()
# mc.set_gripper_calibration()
# time.sleep(3)

# # 감지된 색상을 저장할 변수 선언
# last_detected_color = None

# # HSV 색상 탐지 함수
# def detect_color():
#     cap = cv2.VideoCapture(1)
#     if not cap.isOpened():
#         print("카메라를 열 수 없습니다.")
#         return None

#     # HSV 색상 범위 설정
#     lower_blue = np.array([100, 100, 100])
#     upper_blue = np.array([130, 255, 255])
#     lower_yellow = np.array([20, 100, 100])
#     upper_yellow = np.array([30, 255, 255])
#     lower_green = np.array([35, 100, 100])
#     upper_green = np.array([85, 255, 255])
#     lower_red_1 = np.array([0, 100, 100])
#     upper_red_1 = np.array([10, 255, 255])
#     lower_red_2 = np.array([170, 100, 100])
#     upper_red_2 = np.array([180, 255, 255])

#     start_time = time.time()

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break

#         hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#         blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
#         yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
#         green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
#         red_mask1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
#         red_mask2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
#         red_mask = cv2.add(red_mask1, red_mask2)

#         # 웹캠 화면 표시
#         cv2.imshow("Robot View - Color Detection", frame)
#         cv2.waitKey(1)  # 프레임 표시 대기

#         if cv2.countNonZero(blue_mask) > 0:
#             print("파란색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'blue'
#         elif cv2.countNonZero(yellow_mask) > 0:
#             print("노란색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'yellow'
#         elif cv2.countNonZero(green_mask) > 0:
#             print("초록색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'green'
#         elif cv2.countNonZero(red_mask) > 0:
#             print("빨간색 탐지!")
#             cap.release()
#             cv2.destroyAllWindows()
#             return 'red'

#         if time.time() - start_time > 10:
#             print("10초 동안 색상이 감지되지 않았습니다.")
#             cap.release()
#             cv2.destroyAllWindows()
#             return None
        
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

# # pose0에서 블록 색상 확인
# mc.send_angles([71, -81, 0, 7, 83, 72], 20)
# time.sleep(7)

# # 블록 색상 탐지
# detected_color = detect_color()
# if detected_color:
#     last_detected_color = detected_color  # 블록 색상 기억
#     print(f"탐지된 색상: {detected_color}")
#     mc.set_gripper_state(1, 20, 1)  # 블록을 잡기 위해 그리퍼 닫기
#     time.sleep(2)
#     print("블록을 성공적으로 잡았습니다!")
# else:
#     print("블록의 색상을 감지하지 못했습니다.")

# # pose1부터 pose4까지 이동
# poses = [
#     [60, -50, 0, 10, 85, 60],   # pose1
#     [29, -20, -37, -29, 90, 29], # pose2 yolo로 위치 인식 시작
#     [13, -10, -48, -27, 90, 10], # pose3 
#     [-16, -16, -53, -13, 90, -17] # pose4 yolo로 위치 인식 끝
# ]

# cap = cv2.VideoCapture(1)  # 웹캠 캡처 객체 생성

# for i, pose in enumerate(poses, start=1):
#     print(f"pose{i} 위치로 이동")
#     mc.send_angles(pose, 20)
#     time.sleep(3)  # 각 위치에서 대기 시간 조정

#     if 2 <= i <= 4:  # pose2부터 pose4까지 YOLO 모델 적용
#         ret, frame = cap.read()
#         if not ret:
#             print("프레임을 읽을 수 없습니다.")
#             continue

#         # YOLO 모델을 사용해 객체 인식
#         results = model(frame)
#         print(f"pose{i}에서 인식된 객체:")

#         # 인식된 객체가 잡은 블록과 일치하는지 확인
#         for result in results:
#             if last_detected_color == 'red' and 'red_space' in result.names:
#                 mc.send_angles([-16, -86, -4, -2, 90, -23], 20)
#                 print("빨간색 블록을 지정된 위치에 내려놓습니다.")
#                 break
#             elif last_detected_color == 'blue' and 'blue_space' in result.names:
#                 mc.send_angles([29, -70, -34, -1, 90, 29], 20)
#                 print("파란색 블록을 지정된 위치에 내려놓습니다.")
#                 break
#             elif last_detected_color == 'yellow' and 'yellow_space' in result.names:
#                 mc.send_angles([6, -60, -50, 22, 90, 10], 20)
#                 print("노란색 블록을 지정된 위치에 내려놓습니다.")
#                 break
#             elif last_detected_color == 'green':
#                 mc.send_angles([0, 50, -30, 20, 10, 10], 20)
#                 print("초록색 블록을 지정된 위치에 내려놓습니다.")
#                 break

#         # YOLO 인식 결과 화면 표시
#         plots = results[0].plot()
#         cv2.imshow("YOLO Detection", plots)
#         cv2.waitKey(1)  # 프레임 표시 대기

# cap.release()
# cv2.destroyAllWindows()

# # 작업 완료 후 pose5로 돌아가기
# mc.send_angles([0, 0, 0, 0, 0, 0], 20)
# print("모든 작업을 완료하고 pose5로 복귀합니다.")


from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO
import threading

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# YOLO 모델 로드
model = YOLO('C:\\Users\\shims\\Desktop\\github\\yolo_dectect_traing\\runs\\detect\\train\\weights\\best.pt')

# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)

# 웹캠 스레드로 계속 켜두기 위한 함수
def keep_camera_open():
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # 웹캠 화면 표시
        cv2.imshow("Robot View", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q'를 누르면 종료
            break

    cap.release()
    cv2.destroyAllWindows()

# 웹캠을 계속 켜두기 위해 스레드 시작
camera_thread = threading.Thread(target=keep_camera_open, daemon=True)
camera_thread.start()

# 감지된 색상을 저장할 변수 선언
last_detected_color = None

# HSV 색상 탐지 함수
def detect_color():
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return None

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

    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        red_mask1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
        red_mask2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
        red_mask = cv2.add(red_mask1, red_mask2)

        if cv2.countNonZero(blue_mask) > 0:
            print("파란색 탐지!")
            cap.release()
            return 'blue'
        elif cv2.countNonZero(yellow_mask) > 0:
            print("노란색 탐지!")
            cap.release()
            return 'yellow'
        elif cv2.countNonZero(green_mask) > 0:
            print("초록색 탐지!")
            cap.release()
            return 'green'
        elif cv2.countNonZero(red_mask) > 0:
            print("빨간색 탐지!")
            cap.release()
            return 'red'

        if time.time() - start_time > 10:
            print("10초 동안 색상이 감지되지 않았습니다.")
            cap.release()
            return None

# pose0에서 블록 색상 확인
mc.send_angles([71, -81, 0, 7, 83, 72], 20)
time.sleep(3)

# 블록 색상 탐지
detected_color = detect_color()
if detected_color:
    last_detected_color = detected_color  # 블록 색상 기억
    print(f"탐지된 색상: {detected_color}")
    mc.set_gripper_state(1, 20, 1)  # 블록을 잡기 위해 그리퍼 닫기
    time.sleep(2)
    print("블록을 성공적으로 잡았습니다!")
else:
    print("블록의 색상을 감지하지 못했습니다.")

# pose1부터 pose4까지 이동
poses = [
    [60, -50, 0, 10, 85, 60],   # pose1
    [29, -20, -37, -29, 90, 29], # pose2 yolo로 위치 인식 시작
    [13, -10, -48, -27, 90, 10], # pose3 
    [-16, -16, -53, -13, 90, -17] # pose4 yolo로 위치 인식 끝
]

cap = cv2.VideoCapture(1)  # 웹캠 캡처 객체 생성

for i, pose in enumerate(poses, start=1):
    print(f"pose{i} 위치로 이동")
    mc.send_angles(pose, 20)
    time.sleep(3)  # 각 위치에서 대기 시간 조정

    if 2 <= i <= 4:  # pose2부터 pose4까지 YOLO 모델 적용
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            continue

        # YOLO 모델을 사용해 객체 인식
        results = model(frame)
        print(f"pose{i}에서 인식된 객체:")

        # 인식된 객체가 잡은 블록과 일치하는지 확인
        for result in results:
            if last_detected_color == 'red' and 'red_space' in result.names:
                mc.send_angles([-16, -86, -4, -2, 90, -23], 20)
                print("빨간색 블록을 지정된 위치에 내려놓습니다.")
                break
            elif last_detected_color == 'blue' and 'blue_space' in result.names:
                mc.send_angles([29, -70, -34, -1, 90, 29], 20)
                print("파란색 블록을 지정된 위치에 내려놓습니다.")
                break
            elif last_detected_color == 'yellow' and 'yellow_space' in result.names:
                mc.send_angles([6, -60, -50, 22, 90, 10], 20)
                print("노란색 블록을 지정된 위치에 내려놓습니다.")
                break
            elif last_detected_color == 'green':
                mc.send_angles([0, 50, -30, 20, 10, 10], 20)
                print("초록색 블록을 지정된 위치에 내려놓습니다.")
                break

# 작업 완료 후 pose5로 돌아가기
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
print("모든 작업을 완료하고 pose5로 복귀합니다.")
