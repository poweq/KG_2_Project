from pymycobot.mycobot import MyCobot
import time
import cv2
from ultralytics import YOLO

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# YOLO 모델 로드
model = YOLO('C:\\Users\\shims\\Desktop\\github\\KG_2_Project\\ROOBOTARM_team\\yolov8_model\\runs\\detect\\train2\\weights\\best.pt')

# pose2 위치 설정 (z축과 회전값 고정)
pose2_coords = [204.5, -149.4, 311, -169.63, -2.75, -92.02]
fixed_z = pose2_coords[2]  # z 축 고정

# Z축을 내릴 위치 설정
lowered_z = fixed_z - 300  # 원하는 만큼 z축을 내립니다 (예: 50mm)

# 초기 위치 설정
current_x, current_y = pose2_coords[0], pose2_coords[1]
centered = False  # 중심 맞추기 완료 여부 확인
first_detection = True  # 처음 중심점 위치 출력 여부 확인

# 웹캠 설정
cap = cv2.VideoCapture(1)
CONFIDENCE_THRESHOLD = 0.7
TARGET_X, TARGET_Y = 300, 300  # 목표 좌표를 (389, 333)으로 설정
WINDOW_NAME = "YOLO Detection View"

# 지정된 좌표로 MyCobot을 이동
def move_to_position(x, y, z, rx, ry, rz, speed=20):
    print(f"Moving to position: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    mc.send_coords([x, y, z, rx, ry, rz], speed)

# 빨간 점 인식 후 조정하는 함수
def detect_and_adjust_position():
    global current_x, current_y, centered, first_detection
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        return

    # YOLO 모델 적용
    results = model(frame)
    frame_with_yolo = results[0].plot()

    # 보정 비율 설정 (픽셀 -> 로봇 좌표 변환)
    pixel_to_robot_x = 0.5  # 픽셀 이동에 따른 x축 보정 비율
    pixel_to_robot_y = 0.5  # 픽셀 이동에 따른 y축 보정 비율

    # 빨간 점 인식 후 조정
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

                # 부호 반대로 적용하여 조정값 계산
                adjust_x = (TARGET_X - x_center) * pixel_to_robot_x * (1)  # X축 부호 반전
                adjust_y = (TARGET_Y - y_center) * pixel_to_robot_y * (-1)  # Y축 부호 반전
                print(f"조정값(X좌표 , Y좌표) : ({adjust_x}, {adjust_y})")

                # 새로운 x, y 값을 계산하고 업데이트
                current_x = pose2_coords[0] + adjust_x
                current_y = pose2_coords[1] + adjust_y

                # 로봇을 조정된 좌표로 이동
                move_to_position(current_x, current_y, fixed_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])

                # 중심이 목표 좌표에 근접했는지 확인
                if abs(x_center - TARGET_X) < 10 and abs(y_center - TARGET_Y) < 10:
                    centered = True
                    print("빨간 점이 목표 좌표 근처에 위치했습니다.")
                    return

    # 실시간으로 YOLO Detection View 업데이트
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 600, 600)
    cv2.imshow(WINDOW_NAME, frame_with_yolo)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' 키를 누르면 종료
        cap.release()
        cv2.destroyAllWindows()
        exit()

# Z축을 내리는 함수 추가
def lower_z():
    global current_x, current_y, lowered_z
    print("로봇암을 아래로 내립니다.")
    move_to_position(current_x, current_y, lowered_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])
    time.sleep(5)  # 이동 시간 대기

# MyCobot을 pose2로 이동 후, 빨간 점 인식 후 조정
def main():
    global centered

    # pose2로 이동
    mc.send_angles([-16, -9, -54, -14, 90, -14], 20)
    time.sleep(5)  # pose2에서 10초 대기

    # 중심 맞추기 시작 시간 기록
    start_time = time.time()

    # 빨간 점을 목표 좌표로 맞출 때까지 조정 (최대 15초 동안)
    while not centered and (time.time() - start_time) < 10:
        detect_and_adjust_position()

    # 15초 경과 후 또는 중심 맞추기 완료 후 Z축 내리기
    if not centered:
        print("15초 내에 중심 맞추기에 실패하였습니다. 현재 위치에서 로봇 암을 내립니다.")

    lower_z()

    # 충분한 대기 시간
    time.sleep(10)

    # 작업 완료 후 초기 조인트 위치로 복귀
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    print("모든 작업을 완료하고 초기 조인트 위치로 복귀합니다.")

# 메인 함수 실행
main()
