import cv2
import cv2.aruco as aruco
import math
import json
import websocket
import numpy as np
import time
import socket

# 목표 및 허용 오차
target_angle = 90.0
target_distance = 0.10      # 10cm
angle_threshold = 2.0       # ±3도
distance_threshold = 0.005  # ±0.5cm
angle_hysteresis = 3.0
distance_hysteresis = 0.01

# 마커별 stable 상태
stable_flags = {0: False, 1: False}

# rosbridge 연결
rosbridge_url = "ws://192.168.0.200:9090"
try:
    ws = websocket.create_connection(rosbridge_url)
    print(f"Connected to rosbridge: {rosbridge_url}")
    adv_msg = {
        "op": "advertise",
        "topic": "/pusher_command",
        "type": "std_msgs/String"
    }
    ws.send(json.dumps(adv_msg))
except Exception as e:
    print(f"rosbridge connect error: {e}")
    ws = None

def publish_pusher_command(data_str):
    if ws:
        pub_msg = {
            "op":"publish",
            "topic":"/pusher_command",
            "msg":{"data":data_str},
            "type":"std_msgs/String"
        }
        ws.send(json.dumps(pub_msg))
        print(f"Published '{data_str}' to /pusher_command")
    else:
        print("rosbridge not connected")

def get_camera_matrix():
    return [[389.05937443,0,328.49977251],
            [0,527.33816784,238.71084069],
            [0,0,1]]

def get_dist_coeffs():
    return [0.14130158,-0.32542408,0.00217276,-0.00148743,0.19638934]

camera_matrix = np.array(get_camera_matrix(), dtype=np.float32)
dist_coeffs = np.array(get_dist_coeffs(), dtype=np.float32)

def compute_marker_distance_angle(corners, marker_size):
    rvecs,tvecs,_=aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
    rvec = rvecs[0][0]
    tvec = tvecs[0][0]
    distance = math.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
    angle_to_marker = math.degrees(math.atan2(tvec[0], tvec[2]))
    angle_deg = angle_to_marker+90.0
    return distance, angle_deg, rvec, tvec

# UDP 수신 설정
UDP_IP = "192.168.0.200"  #  PC IP
UDP_PORT = 5005
MAX_PACKET_SIZE = 60000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("윈도우: UDP로 카메라 데이터 수신 대기 중...")
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
marker_size = 0.055

buffer = b""

while True:
    # 패킷 수신
    packet, addr = sock.recvfrom(MAX_PACKET_SIZE)
    buffer += packet

    # JPEG 끝(FFD9) 확인
    if len(packet) < MAX_PACKET_SIZE or buffer[-2:] == b'\xff\xd9':
        # JPEG 디코딩
        frame = cv2.imdecode(np.frombuffer(buffer, dtype=np.uint8), cv2.IMREAD_COLOR)
        buffer = b""

        if frame is None:
            print("JPEG 디코딩 실패")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

        found_ids = set()
        if ids is not None and len(ids)>0:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id in [0,1]:
                    found_ids.add(marker_id)
                    distance, angle_deg, rvec, tvec = compute_marker_distance_angle(corners[i], marker_size)
                    angle_diff = angle_deg - target_angle
                    dist_err = distance - target_distance

                    # 마커 테두리 표시
                    aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])
                    axis_length = marker_size * 0.5
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, axis_length)

                    cv2.putText(frame, f"ID:{marker_id} Angle:{angle_deg:.2f}deg",(10,30),
                                cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
                    cv2.putText(frame, f"Distance:{distance:.2f}m",(10,70),
                                cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)

                    print(f"ID:{marker_id} Dist={distance:.3f},Ang={angle_deg:.3f},Diff={angle_diff:.3f},DistErr={dist_err:.3f},stable={stable_flags[marker_id]}")

                    if stable_flags[marker_id]:
                        # stable 상태 유지 혹은 해제
                        if abs(angle_diff)>angle_threshold+angle_hysteresis or abs(dist_err)>distance_threshold+distance_hysteresis:
                            print(f"ID:{marker_id} Stable 해제조건 angle_diff={angle_diff}, dist_err={dist_err}")
                            stable_flags[marker_id] = False
                        else:
                            print(f"ID:{marker_id} Stable 유지 상태")
                    else:
                        # stable False → 정지 조건 확인
                        if abs(angle_diff)<angle_threshold and abs(dist_err)<distance_threshold:
                            print(f"ID:{marker_id} 정지조건 충족 angle_diff={angle_diff}, dist_err={dist_err}")
                            stable_flags[marker_id] = True
                            # 마커 ID별 퍼블리시 메시지
                            if marker_id == 0:
                                publish_pusher_command("x:1")
                            elif marker_id == 1:
                                publish_pusher_command("y:1")
                            print(f"ID:{marker_id} 조건 만족: 토픽 퍼블리시 완료, stable=True로 전환")
                        else:
                            print(f"ID:{marker_id} stable조건 미충족 angle_diff={angle_diff}, dist_err={dist_err}")
        else:
            print("No Marker detected")

        # 마커가 사라진 경우 stable_flag 해제
        for mid in [0,1]:
            if mid not in found_ids and stable_flags[mid]:
                print(f"ID:{mid} No Marker but stable=True: 마커 잃었으므로 stable 해제")
                stable_flags[mid] = False

        cv2.imshow("UDP Video Stream", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27: # ESC
            break

sock.close()
cv2.destroyAllWindows()