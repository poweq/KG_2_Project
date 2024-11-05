from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# YOLO 모델 로드
model = YOLO('C:\\Users\\shims\\Desktop\\github\\KG_2_Project\\ROOBOTARM_team\\yolov8_detect_model\\runs\\detect\\train2\\weights\\best.pt')

# pose2 위치 설정 (z축과 회전값 고정)
pose2_coords = [245.6, -64.8, -40.2, 178.07, -0.15, -83.39]
fixed_z = pose2_coords[2]  # z 축 고정

# 초기 위치 설정
current_x, current_y = pose2_coords[0], pose2_coords[1]
centered = False  # 중심 맞추기 완료 여부 확인
first_detection = True  # 처음 중심점 위치 출력 여부 확인

# 웹캠 설정
cap = cv2.VideoCapture(1)
CONFIDENCE_THRESHOLD = 0.7
TARGET_X, TARGET_Y = 300, 300
WINDOW_NAME = "YOLO Detection View"

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
    cap = cv2.VideoCapture(1)
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
            print("파란색 블록 탐지!")
            return 'blue'
        elif cv2.countNonZero(yellow_mask) > 0:
            print("노란색 블록 탐지!")
            return 'yellow'
        elif cv2.countNonZero(green_mask) > 0:
            print("초록색 블록 탐지!")
            return 'green'
        elif cv2.countNonZero(red_mask) > 0:
            print("빨간색 블록 탐지!")
            return 'red'

        if time.time() - start_time > 10:
            print("10초 동안 색상이 감지되지 않았습니다.")
            return None

last_detected_color = None  # 전역 변수로 블록 색상 기억

# pose0에서 블록 색상 확인 및 잡기
def detect_and_grab_block():
    global last_detected_color
    mc.send_angles([71, -81, 0, 7, 83, 72], 20)  # pose0 위치로 이동
    time.sleep(3)
    
    detected_color = detect_color()
    if detected_color:
        last_detected_color = detected_color  # 블록 색상 기억
        print(f"탐지된 색상: {detected_color}")
        mc.set_gripper_state(1, 20, 1)  # 그리퍼로 블록 잡기
        time.sleep(2)
        print("블록을 성공적으로 잡았습니다!")
    else:
        print("블록의 색상을 감지하지 못했습니다.")

# pose2에서 객체인식 후 중점(빨간 점)을 인식하고 조정
def perform_pose2_adjustments():
    # pose2로 이동
    mc.send_angles([6, -13, -51, -26, 90, 2], 20)
    time.sleep(10)

# 빨간 점을 화면 중앙으로 맞출 때까지 조정
    while not centered:
        detect_and_adjust_position()

def move_to_position(x, y, z, rx, ry, rz, speed=20):
    print(f"Moving to position: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    mc.send_coords([x, y, z, rx, ry, rz], speed)

def detect_and_adjust_position():
    global current_x, current_y, centered, first_detection
    start_time = time.time()  # 함수 시작 시간 기록
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        return

    # YOLO 모델 적용
    results = model(frame)
    frame_with_yolo = results[0].plot()

    # 보정 비율 설정 (픽셀 -> 로봇 좌표 변환)
    pixel_to_robot_x = 0.2  # x축 보정 비율을 더 작게 설정하여 세밀한 조정
    pixel_to_robot_y = 0.2  # y축 보정 비율을 더 작게 설정하여 세밀한 조정

    # 빨간 점 인식 후 조정
    while True:
        # 15초 경과 시 함수 종료
        if time.time() - start_time > 10:
            print("10초가 경과하여 함수가 종료됩니다.")
            centered = True  # 타임아웃 시 중심에 도달한 것으로 처리
            return

        for result in results:
            for box in result.boxes:
                if box.conf >= CONFIDENCE_THRESHOLD:
                    # 바운딩 박스의 중심 계산
                    x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                    x_center = (x_min + x_max) // 2
                    y_center = (y_min + y_max) // 2

                    cv2.circle(frame_with_yolo, (x_center, y_center), 5, (0, 0, 255), -1)

                    # 중심점 위치 출력
                    if first_detection:
                        print(f"중심점 위치(X좌표 , Y좌표) : ({x_center}, {y_center})")
                        first_detection = False

                    # 중심을 화면 중앙으로 맞추기 위해 x, y 값만 조정
                    adjust_x = (TARGET_X - x_center) * pixel_to_robot_x
                    adjust_y = (TARGET_Y - y_center) * pixel_to_robot_y
                    print(f"조정값(X좌표 , Y좌표) : ({adjust_x}, {adjust_y})")

                    # 새로운 x, y 값을 계산하고 업데이트
                    current_x = pose2_coords[0] + adjust_x
                    current_y = pose2_coords[1] + adjust_y

                    # 로봇을 조정된 좌표로 이동
                    move_to_position(current_x, current_y, fixed_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])

                    # 중심이 거의 중앙에 맞았다고 판단하면 centered를 True로 설정
                    print(f"현재 중심 좌표: ({x_center}, {y_center}), 목표 좌표: ({TARGET_X}, {TARGET_Y})")
                    if abs(x_center - TARGET_X) < 10 and abs(y_center - TARGET_Y) < 10:
                        centered = True  # 중심 도달 표시
                        print("중심이 화면 중앙에 맞춰졌습니다.")
                        return  # 함수 종료

        # 실시간으로 YOLO Detection View 업데이트
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 600, 600)
        cv2.imshow(WINDOW_NAME, frame_with_yolo)

        if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' 키를 누르면 종료
            cap.release()
            cv2.destroyAllWindows()
            exit()

# 블록 색상에 따라 최종 위치를 조정하는 함수
def block_box_match():
    # 현재 위치 (detect_and_adjust_position 함수의 마지막 위치)
    x = current_x
    y = current_y
    z = fixed_z
    rx, ry, rz = pose2_coords[3], pose2_coords[4], pose2_coords[5]

    # 블록 색상에 따른 y 좌표 조정
    if last_detected_color == 'blue':
        y += 30  # 파란색일 경우 y 값에 30 추가
        print("블록이 파란색입니다. y 값을 30 추가합니다.")
    elif last_detected_color == 'yellow':
        y -= 30  # 노란색일 경우 y 값에서 30 빼기
        print("블록이 노란색입니다. y 값을 30 빼기 합니다.")

    # 최종 위치로 로봇 이동
    print(f"블록을 놓는 위치로 이동: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    move_to_position(x, y, z, rx, ry, rz)

# 메인 함수
def main():
    detect_and_grab_block()  # pose0에서 블록 색상 확인 및 잡기

    mc.send_angles([60, -50, 0, 10, 85, 60], 20)

    perform_pose2_adjustments()  # pose2로 이동 후 빨간 점 인식 및 조정
    
    block_box_match()  # 블록 색상에 따른 위치 조정

    if centered:
        print("그리퍼가 열립니다.")
        mc.set_gripper_state(0, 20, 1)  # 그리퍼 열기
    
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)  # pose3 작업 완료 후 돌아가기
    print("모든 작업을 완료하고 복귀합니다.")

# 메인 함수 실행
main()
