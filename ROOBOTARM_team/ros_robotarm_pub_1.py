from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO
import socket
import threading
import websocket
import json

class ROS2WebSocketClient:
    def __init__(self, rosbridge_ip='192.168.0.200', rosbridge_port=9090):
        self.rosbridge_ip = rosbridge_ip
        self.rosbridge_port = rosbridge_port
        self.ws = None
        self.topics = {}
        self.advertised_topics = set()
        self.is_connected = False

    def connect(self):
        websocket.enableTrace(False)
        ws_url = f"ws://{self.rosbridge_ip}:{self.rosbridge_port}"
        self.ws = websocket.WebSocketApp(
            ws_url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        # 별도의 스레드에서 웹소켓 실행
        self.thread = threading.Thread(target=self.ws.run_forever)
        self.thread.daemon = True
        self.thread.start()
        # 연결 완료될 때까지 대기
        while not self.is_connected:
            time.sleep(0.1)

    def on_open(self, ws):
        print("Connected to ROS 2 bridge server.")
        self.is_connected = True

    def on_message(self, ws, message):
        data = json.loads(message)
        if data.get('op') == 'publish':
            topic = data.get('topic')
            msg = data.get('msg')
            if topic in self.topics:
                callback = self.topics[topic]
                callback(msg)

    def on_error(self, ws, error):
        print("WebSocket error:", error)

    def on_close(self, ws, close_status_code, close_msg):
        print("Disconnected from ROS 2 bridge server.")
        self.is_connected = False

    def advertise_topic(self, topic, msg_type):
        """토픽을 광고하여 메시지 타입을 지정"""
        if topic not in self.advertised_topics:
            advertise_message = {
                "op": "advertise",
                "topic": topic,
                "type": msg_type
            }
            self.ws.send(json.dumps(advertise_message))
            self.advertised_topics.add(topic)
            print(f"Advertised topic: {topic} with type: {msg_type}")

    def unadvertise_topic(self, topic):
        """토픽 광고 해제"""
        if topic in self.advertised_topics:
            unadvertise_message = {
                "op": "unadvertise",
                "topic": topic
            }
            self.ws.send(json.dumps(unadvertise_message))
            self.advertised_topics.remove(topic)
            print(f"Unadvertised topic: {topic}")

    def send_data(self, topic, message, msg_type='std_msgs/String'):
        """ROS 2 토픽에 데이터 퍼블리시"""
        # 토픽이 광고되지 않았으면 광고
        if topic not in self.advertised_topics:
            self.advertise_topic(topic, msg_type)

        publish_message = {
            "op": "publish",
            "topic": topic,
            "msg": {
                "data": message
            }
        }
        self.ws.send(json.dumps(publish_message))

    def receive_data(self, topic, callback, msg_type='std_msgs/String'):
        """ROS 2 토픽을 구독하고 수신한 데이터를 콜백 함수로 처리"""
        self.topics[topic] = callback
        subscribe_message = {
            "op": "subscribe",
            "topic": topic,
            "type": msg_type
        }
        self.ws.send(json.dumps(subscribe_message))
        print(f"Subscribed to topic: {topic} with type: {msg_type}")

    def unsubscribe_data(self, topic):
        """토픽 구독 해제"""
        if topic in self.topics:
            unsubscribe_message = {
                "op": "unsubscribe",
                "topic": topic
            }
            self.ws.send(json.dumps(unsubscribe_message))
            del self.topics[topic]
            print(f"Unsubscribed from topic: {topic}")

    def disconnect(self):
        # 광고된 모든 토픽 광고 해제
        for topic in list(self.advertised_topics):
            self.unadvertise_topic(topic)
        self.ws.close()
        self.thread.join()

# 글로벌 변수 선언
mc = MyCobot('/dev/ttyACM0', 115200)  # 로봇 포트와 보드레이트는 환경에 맞게 설정하세요
ros_client = None
cap = None
running = False
signal_received = None
task_thread = None
should_exit = False
a_detected_qr_list = []  # A 데이터 탐지 순서 저장
b_detected_qr_list = []  # B 데이터 탐지 순서 저장
current_x = 0
current_y = 0

# 로봇 연결 확인 함수
def check_robot_connection():
    global mc
    try:
        mc.get_angles()
        print("MyCobot 연결 성공")
        return True
    except Exception as e:
        print(f"MyCobot 연결 실패: {e}")
        return False

# 카메라 초기화 함수
def init_camera():
    global cap
    if cap is not None and cap.isOpened():
        print("카메라가 이미 초기화되어 있습니다.")
        return cap

    cap = cv2.VideoCapture(1)
    if cap.isOpened():
        print("카메라 초기화 성공.")
        return cap
    else:
        print("카메라 초기화 실패.")
        cap = None
        return None

def release_camera():
    global cap
    if cap is not None:
        cap.release()  # 카메라 리소스 해제
        cv2.destroyAllWindows()
        cap = None
        print("카메라가 닫혔습니다.")

# QR 코드 인식 함수 (생략 - 기존 코드 유지)

# QR 코드 인식 함수
def detect_qr_code():
    global cap
    if cap is None or not cap.isOpened():
        print("카메라가 초기화되지 않았습니다. 다시 초기화 시도 중...")
        cap = init_camera()
        if cap is None:
            print("카메라 재초기화 실패.")
            return None

    for attempt in range(10):  # QR 코드 감지를 최대 10회 시도
        ret, frame = cap.read()
        if not ret:
            print("카메라에서 프레임을 가져올 수 없습니다. 다시 시도 중...")
            cap = init_camera()
            continue

        print("QR 코드 감지 시도 중...")
        detector = cv2.QRCodeDetector()

        # 밝기 및 대비 조정
        for alpha in [1.0, 1.2, 1.5]:
            for beta in [0, 30, 60]:
                adjusted_frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
                data, points, _ = detector.detectAndDecode(adjusted_frame)
                if points is not None and data:
                    print(f"QR 코드 인식 성공: {data}")
                    return data

        print("QR 코드 감지 실패, 재시도...")
    print("QR 코드 감지 실패. 모든 시도 종료.")
    return None

# QR 코드 URL과 전송할 데이터 매핑 딕셔너리
qr_map = {
    'https://site.naver.com/patient/A_1': 'A_1',
    'https://site.naver.com/patient/A_2': 'A_2',
    'https://site.naver.com/patient/A_3': 'A_3',
    'https://site.naver.com/patient/B_1': 'B_1',
    'https://site.naver.com/patient/B_2': 'B_2',
    'https://site.naver.com/patient/B_3': 'B_3'
}

def detect_and_store_qr_codes():
    """
    QR 코드를 탐지하고 A/B 데이터를 분류해 저장합니다.
    """
    global a_detected_qr_list, b_detected_qr_list

    for attempt in range(10):  # QR 코드 감지를 최대 10회 시도
        detected_qr = detect_qr_code()
        if detected_qr:
            print(f"탐지된 QR 코드: {detected_qr}")

            # A 데이터인지 B 데이터인지 확인하고 저장
            if 'A_' in detected_qr and detected_qr not in a_detected_qr_list:
                a_detected_qr_list.append(detected_qr)
                print(f"A 데이터 순서 업데이트: {a_detected_qr_list}")
            elif 'B_' in detected_qr and detected_qr not in b_detected_qr_list:
                b_detected_qr_list.append(detected_qr)
                print(f"B 데이터 순서 업데이트: {b_detected_qr_list}")

            # QR 코드 매핑 딕셔너리를 통해 서버로 데이터 전송
            if detected_qr in qr_map:
                send_value = qr_map[detected_qr]
                ros_client.send_data('/qr_data', send_value, msg_type='std_msgs/String')
                print(f"서버로 '{send_value}' 데이터 전송 완료.")
            else:
                print(f"QR 코드 매핑에서 '{detected_qr}'를 찾을 수 없습니다.")

# pose0에서 QR 코드 인식 및 블록 잡기
def detect_and_grab_block():
    """
    QR 코드를 탐지하고 블록을 잡습니다.
    """
    global a_detected_qr_list, b_detected_qr_list

    # pose0로 이동
    mc.send_angles([-20, 7.29, 81.03, 1.66, -90, 70], 20) # pose0_1 웹캠으로 QR 코드 확인 위치
    time.sleep(5)

    # QR 코드 감지 및 분류
    detect_and_store_qr_codes()

    # 블록 잡기 (A 데이터와 B 데이터를 모두 탐지한 경우)
    if a_detected_qr_list or b_detected_qr_list:
        mc.send_angles([-23, 3.29, 81.03, 1.66, -90, 70], 20)
        time.sleep(1)        
        mc.send_angles([-26.54, 15.55, 64.68, 14.5, -93.6, 64.68], 20)
        time.sleep(1)
        mc.send_angles([-25.4, 21.53, 88.33, -14.94, -93.51, 65.83], 20)  # pose0_3 그리퍼로 블록 잡는 위치
        time.sleep(3)
        mc.set_gripper_mode(0)
        mc.init_gripper()
        mc.init_eletric_gripper()
        mc.set_gripper_value(15, 20, 1) # 그리퍼로 블록 잡기
        time.sleep(3)
        mc.send_angles([-25.48, 34.8, 14.32, 48.42, -93.33, 65.65], 20)
        time.sleep(3)
        print("블록을 성공적으로 잡았습니다!")
        return True


    print(f"QR 코드 감지 실패, {attempt + 1}번째 시도 완료.")
    time.sleep(2)  # 재시도 전 대기    

    print("QR 코드를 감지하지 못했습니다. 작업 실패.")
    return False

# pose2에서 객체 인식 후 조정
def perform_pose2_adjustments():
    global centered
    centered = False  # 조정 시작 시 초기화
    mc.send_angles([90, 17.05, 17.75, 42.46, -90, 2], 20)  # pose2 위치로 이동
    time.sleep(5)

    # 중심 맞추기 시작 시간 기록
    start_time = time.time()
    time_limit = 10  # 10초 제한

    print("객체 중심 맞추기를 시작합니다.")

    while not centered and (time.time() - start_time) < time_limit:
        detect_and_adjust_position()

    if not centered:
        print(f"{time_limit}초 내에 현재 위치에서 로봇 암을 내립니다.")
    else:
        print("빨간 점이 목표 좌표 근처에 위치했습니다.")

def move_to_position(x, y, z, rx, ry, rz, speed=20):
    print(f"Moving to position: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    mc.send_coords([x, y, z, rx, ry, rz], speed)
    time.sleep(2)  # 명령 실행 시간을 줌
    coords = mc.get_coords()
    print(f"현재 로봇 위치: {coords}")

# 객체 감지 및 위치 조정 함수
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

    # # 실시간으로 YOLO Detection View 업데이트
    # cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    # cv2.resizeWindow(WINDOW_NAME, 600, 600)
    # cv2.imshow(WINDOW_NAME, frame_with_yolo)

    # if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' 키를 누르면 종료
    #     cap.release()
    #     cv2.destroyAllWindows()
    #     exit()

def lower_z():
    global current_x, current_y, lowered_z, lowered_y
    lowered_z = fixed_z - 150
    lowered_y = current_y - 30

    print("로봇암을 아래로 내립니다.")
    move_to_position(current_x, lowered_y, lowered_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])
    time.sleep(5)

def handle_data(data_list, direction):
    """
    QR 데이터를 탐지된 순서에 따라 처리합니다.
    - direction: "A" -> positive, "B" -> negative
    """
    global current_x, current_y

    # 이동 방향 설정
    x_step = 60 if direction == "A" else -60

    for idx, qr_code in enumerate(data_list):
        # 데이터 순서에 따라 좌표 계산
        if idx == 0:
            x = current_x + x_step
            y = current_y + 70
        elif idx == 1:
            x = current_x + x_step
            y = current_y
        elif idx == 2:
            x = current_x + x_step
            y = current_y - 80
        else:
            print(f"예상하지 못한 {direction} 데이터 순서: {qr_code}, 스킵합니다.")
            continue

        z = mc.get_coords()[2]
        rx, ry, rz = pose2_coords[3], pose2_coords[4], pose2_coords[5]
        print(f"{direction} 블록 배치 위치 (QR: {qr_code}) -> x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
        move_to_position(x, y, z, rx, ry, rz)
        mc.set_gripper_value(100, 20, 1)  # 그리퍼 열기
        print(f"{direction} 블록 놓기 완료 (QR: {qr_code}).")
        time.sleep(5)

        # x, y 기준 업데이트
        current_x, current_y = x, y

def handle_a_data():
    """
    A 데이터를 탐지된 순서에 따라 처리합니다.
    """
    global a_detected_qr_list, current_x, current_y

    for idx, qr_code in enumerate(a_detected_qr_list):
        if idx == 0:  # 첫 번째로 온 A 데이터
            x = current_x + 60
            y = current_y + 70
        elif idx == 1:  # 두 번째로 온 A 데이터
            x = current_x + 60
            y = current_y
        elif idx == 2:  # 세 번째로 온 A 데이터
            x = current_x + 60
            y = current_y - 80
        else:
            print(f"예상하지 못한 A 데이터 순서: {qr_code}, 스킵합니다.")
            continue

        z = mc.get_coords()[2]
        rx, ry, rz = pose2_coords[3], pose2_coords[4], pose2_coords[5]
        print(f"A 블록 배치 위치 (QR: {qr_code}) -> x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
        move_to_position(x, y, z, rx, ry, rz)
        mc.set_gripper_value(100, 20, 1)  # 그리퍼 열기
        print(f"A 블록 놓기 완료 (QR: {qr_code}).")
        time.sleep(5)

        # x, y 기준 업데이트
        current_x, current_y = x, y

def handle_b_data():
    """
    B 데이터를 탐지된 순서에 따라 처리합니다.
    """
    global b_detected_qr_list, current_x, current_y

    for idx, qr_code in enumerate(b_detected_qr_list):
        if idx == 0:  # 첫 번째로 온 B 데이터
            x = current_x - 60
            y = current_y + 70
        elif idx == 1:  # 두 번째로 온 B 데이터
            x = current_x - 60
            y = current_y
        elif idx == 2:  # 세 번째로 온 B 데이터
            x = current_x - 60
            y = current_y - 80
        else:
            print(f"예상하지 못한 B 데이터 순서: {qr_code}, 스킵합니다.")
            continue

        z = mc.get_coords()[2]
        rx, ry, rz = pose2_coords[3], pose2_coords[4], pose2_coords[5]
        print(f"B 블록 배치 위치 (QR: {qr_code}) -> x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
        move_to_position(x, y, z, rx, ry, rz)
        mc.set_gripper_value(100, 20, 1)  # 그리퍼 열기
        print(f"B 블록 놓기 완료 (QR: {qr_code}).")
        time.sleep(5)

        # x, y 기준 업데이트
        current_x, current_y = x, y

def process_qr_data():
    """
    저장된 A와 B 데이터를 각각 처리합니다.
    """
    if a_detected_qr_list:
        print("A 데이터 처리 시작...")
        handle_data(a_detected_qr_list, "A")
    if b_detected_qr_list:
        print("B 데이터 처리 시작...")
        handle_data(b_detected_qr_list, "B")

def reset_robot():
    #robot_arm_node.publish_pose("reset...")  # reset
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    time.sleep(5)
    mc.set_gripper_value(100,20,1)  # 그리퍼 열기
    time.sleep(3)
    print("로봇이 초기 위치로 돌아갔습니다.")

# 기타 로봇 동작 함수들 (detect_and_grab_block, perform_pose2_adjustments 등은 기존 코드 유지)

# 로봇 작업 함수 (별도의 스레드에서 실행)
# def robot_task():
    global running, should_exit, ros_client
    try:
        if not running or should_exit:
            return
        
        # 1단계: pose 1 퍼블리시
        ros_client.send_data('/robot_arm_status', 'pose 1', msg_type='std_msgs/String')
        print("Published: pose 1")
        if not running or should_exit:
            return

        # QR 코드 감지 및 블록 잡기
        if detect_and_grab_block():
            if not running or should_exit:
                return
            
            print("객체 중심 맞추기...")

            # 2단계: pose 2 퍼블리시
            ros_client.send_data('/robot_arm_status', 'pose 2', msg_type='std_msgs/String')
            print("Published: pose 2")
            if not running or should_exit:
                return

            # A와 B 데이터를 처리
            process_qr_data()
            if not running or should_exit:
                return
            
            # 3단계: pose 3 퍼블리시
            ros_client.send_data('/robot_arm_status', 'pose 3', msg_type='std_msgs/String')
            print("Published: pose 3")
            if not running or should_exit:
                return
            
            print("Z축 내리기...")
            lower_z()
            if not running or should_exit:
                return

            print("블록 배치...")
            
            # 로봇 초기 위치 복귀
            reset_robot()
            time.sleep(7)
            if not running or should_exit:
                return

            ros_client.send_data('/robot_arm_status', 'pose 0', msg_type='std_msgs/String')
            print("Published: pose 0")
            if not running or should_exit:
                return

        else:
            print("QR 코드 감지 실패 또는 블록 잡기 실패. 작업을 종료합니다.")
            reset_robot()
            # 여기서도 stop 신호 확인 가능
            if not running or should_exit:
                return

    finally:
        running = False


def robot_task():
    global running, last_detected_qr, should_exit, ros_client
    try:
        # 시작 시 running과 should_exit 상태 체크
        if not running or should_exit:
            return  # 실행 중단

        # 1단계: pose 1 퍼블리시
        ros_client.send_data('/robot_arm_status', 'pose 1', msg_type='std_msgs/String')
        print("Published: pose 1")
        if not running or should_exit:
            return

        # 블록 감지 및 잡기 시도
        if detect_and_grab_block():
            if not running or should_exit:
                return

            print("객체 중심 맞추기...")

            # 2단계: pose 2 퍼블리시
            ros_client.send_data('/robot_arm_status', 'pose 2', msg_type='std_msgs/String')
            print("Published: pose 2")
            if not running or should_exit:
                return

            perform_pose2_adjustments()  # 감지 실패해도 다음 단계로 진행
            if not running or should_exit:
                return

            # 3단계: pose 3 퍼블리시
            ros_client.send_data('/robot_arm_status', 'pose 3', msg_type='std_msgs/String')
            print("Published: pose 3")
            if not running or should_exit:
                return

            print("Z축 내리기...")
            lower_z()
            if not running or should_exit:
                return

            print("블록 배치...")

            # 4단계: pose 4 퍼블리시
            ros_client.send_data('/robot_arm_status', 'pose 4', msg_type='std_msgs/String')
            print("Published: pose 4")
            if not running or should_exit:
                return

            process_qr_data()
            if not running or should_exit:
                return

            # 그리퍼 열기
            time.sleep(4)
            mc.set_gripper_state(0, 20, 1)
            print("그리퍼 열기...")
            if not running or should_exit:
                return

            # 로봇 초기 위치 복귀
            reset_robot()
            time.sleep(7)
            if not running or should_exit:
                return
            
            # 지금까지 들어온 데이터를 출력
            print("\n===== 작업 완료: QR 코드 데이터 리스트 =====")
            print(f"A 데이터 리스트: {a_detected_qr_list}")
            print(f"B 데이터 리스트: {b_detected_qr_list}")
            print("========================================\n")

            # 5단계: pose 0 퍼블리시
            ros_client.send_data('/robot_arm_status', 'pose 0', msg_type='std_msgs/String')
            print("Published: pose 0")
            if not running or should_exit:
                return

        else:
            print("QR 코드 감지 실패 또는 블록 잡기 실패. 작업을 종료합니다.")
            reset_robot()
            # 여기서도 stop 신호 확인 가능
            if not running or should_exit:
                return

    finally:
        running = False  # 작업 종료 표시

# 로봇 시작 함수
def start_robot():
    global running, task_thread
    if not running:
        running = True
        task_thread = threading.Thread(target=robot_task)
        task_thread.start()
        print("Robot task started.")
    else:
        print("Robot is already running.")

def stop_robot():
    global running, task_thread
    if running:
        running = False
        print("Stopping robot...")
        # 현재 동작 중인 task_thread가 종료되기를 기다림
        if task_thread and task_thread.is_alive():
            task_thread.join()
        print("Robot stopped.")
        
        # 로봇을 초기 자세로 복귀
        reset_robot()
        print("로봇이 초기 위치로 돌아갔습니다.")
        # pose0 코드 추가
        ros_client.send_data('/robot_arm_status', 'pose 0', msg_type='std_msgs/String')
        print("Published: pose 0")
    else:
        print("Robot is not running.")

# 메인 함수
def main():
    global cap, should_exit, signal_received, ros_client

    # 클래스 인스턴스 생성
    ros_client = ROS2WebSocketClient(rosbridge_ip='192.168.0.200', rosbridge_port=9090)

    # ROS 2 브릿지 서버에 연결
    ros_client.connect()

    # 수신 데이터 처리 콜백 함수 정의
    def robot_command_callback(msg):
        command = msg.get('data')
        print(f"Received command: {command}")
        if command == 'start':
            start_robot()
        elif command == 'stop':
            stop_robot()
        else:
            print(f"Unknown command: {command}")

    # '/robot_arm_command' 토픽 구독 (메시지 타입 지정)
    ros_client.receive_data('/robot_arm_command', robot_command_callback, msg_type='std_msgs/String')

    # 로봇 연결 확인
    if not check_robot_connection():
        print("MyCobot 연결 실패. 프로그램을 종료합니다.")
        return

    print("MyCobot 초기화 중...")
    reset_robot()
    time.sleep(2)
    print("초기화 완료.")

    #pose0 코드 추가
    ros_client.send_data('/robot_arm_status', 'pose 0', msg_type='std_msgs/String')
    print("Published: pose 0")
    
    ros_client.send_data('/qr_data', "no data", msg_type='std_msgs/String')
    print("QR 토픽데이터 초기화")

    ros_client.send_data('/agv_command', "start", msg_type='std_msgs/String')
    print("AGV 시작문자 전송")

    # 카메라 초기화
    cap = init_camera()
    if cap is None:  # 초기화 실패 시 종료
        print("카메라 초기화 실패. 프로그램을 종료합니다.")
        return

    # 메인 루프
    try:
        while not should_exit:
            time.sleep(1)  # 프로그램 유지
    except KeyboardInterrupt:
        should_exit = True
    finally:
        # 리소스 해제
        if cap:
            release_camera()
        if ros_client:
            ros_client.disconnect()
        print("프로그램 종료.")

# YOLO 모델 로드
model = YOLO('/home/shim/github/KG_2_Project/ROOBOTARM_team/yolov8_model/runs/detect/train6/weights/best.pt')

# 글로벌 변수(전역 변수) 선언
running = False            # 로봇 작업 진행 상태
signal_received = None     # 신호 상태
task_thread = None         # 로봇 작업 스레드정
should_exit = False        # 프로그램 종료 플래그 추가
last_detected_qr = None    # QR 코드 데이터 저장
cap = None                 # 카메라 객체

# 픽셀-로봇 좌표 변환 비율 설정
pixel_to_robot_x = 0.2
pixel_to_robot_y = 0.2

# pose2 위치 (z축 고정)
pose2_coords = [86.9, -219.2, 352.8, -169.02, 0.54, 177.87]
fixed_z = pose2_coords[2]

# 초기 변수 설정
current_x, current_y = pose2_coords[0], pose2_coords[1]
centered = False           # 중심 맞추기 완료 여부 확인
first_detection = True     # 처음 중심점 위치 출력 여부 확인

CONFIDENCE_THRESHOLD = 0.7
TARGET_X, TARGET_Y = 300, 300
WINDOW_NAME = "YOLO Detection View"

if __name__ == "__main__":
    main()