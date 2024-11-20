#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial
import threading
import struct
import math
from rclpy.qos import qos_profile_sensor_data

class AGVController(Node):
    def __init__(self):
        super().__init__('agv_controller_node')
        
        # 속도 제한 설정
        self.limit_speed = 80  # 최대 속도 제한값

        # 시리얼 포트 설정 (Raspberry Pi의 ttyAMA1 포트를 사용, 통신 속도 9600)
        try:
            self.serial_port = serial.Serial('/dev/ttyAMA1', 9600, timeout=1)
            self.get_logger().info("Serial connection established on /dev/ttyAMA1 at 9600 baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.serial_port = None

        # /cmd_vel 토픽 구독
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # /imu/data 토픽 퍼블리셔
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)

        # 시리얼 수신 스레드 시작
        if self.serial_port:
            self.serial_thread = threading.Thread(target=self.read_serial_data)
            self.serial_thread.daemon = True
            self.serial_thread.start()

    def cmd_vel_callback(self, msg):
        if not self.serial_port:
            self.get_logger().error("Serial port not available.")
            return

        # linear.x 값과 angular.z 값을 추출
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 모터 속도 리스트 초기화
        motor_command = []

        # 분기점 설정
        if linear_x == 0 and angular_z != 0:
            # 제자리 회전 모드
            rotation_speed = max(min(int(abs(angular_z) * self.limit_speed), self.limit_speed), -self.limit_speed)

            if angular_z < 0:
                # 왼쪽 회전, ccw 방향
                motor_command = [0xf1, 'cc', rotation_speed]
            elif angular_z > 0:
                # 오른쪽 회전, cw 방향
                motor_command = [0xf1, 'cw', rotation_speed]
        else:
            # 기존 동작 모드 (linear.x와 angular_z 값을 모두 고려)
            base_speed = max(min(int(abs(linear_x) * self.limit_speed), self.limit_speed), -self.limit_speed)
            motor_command = [0xf0, base_speed, base_speed, base_speed, base_speed]

            # 감속 및 가속 비율 적용
            reduction = int(abs(base_speed) * 0.55)  # 55% 감속
            boost = int(abs(base_speed) * 0.15)  # 15% 가속

            # angular_z에 따라 감속 및 가속 조정
            if angular_z < 0:
                motor_command[1] -= reduction  # c 감속
                motor_command[2] -= reduction  # d 감속
                motor_command[3] += boost  # a 가속
                motor_command[4] += boost  # b 가속
            elif angular_z > 0:
                motor_command[1] += boost  # c 가속
                motor_command[2] += boost  # d 가속
                motor_command[3] -= reduction  # a 감속
                motor_command[4] -= reduction  # b 감속

            # 후진 시 부호 반전
            if linear_x < 0:
                motor_command[1] = -motor_command[1]
                motor_command[2] = -motor_command[2]
                motor_command[3] = -motor_command[3]
                motor_command[4] = -motor_command[4]

        # 리스트 기반으로 명령 문자열 생성 및 전송
        motor_command_str = ','.join(map(str, motor_command)) + '\n'
        self.serial_port.write(motor_command_str.encode())

        # 로그에 리스트와 각 모터에 전송되는 명령을 출력
        self.get_logger().info(
            f"Linear X: {linear_x}, Angular Z: {angular_z}\n"
            f"Adjusted Motor Command List: {motor_command}\n"
            f"Commands sent to motors: {motor_command}"
        )

    def read_serial_data(self):
        buffer = bytearray()
        while self.serial_port.is_open:
            try:
                # 읽을 데이터가 있는지 확인
                if self.serial_port.in_waiting > 0:
                    # 데이터 읽기
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer += data

                    while True:
                        if len(buffer) < 19:
                            # 패킷 크기 미달 시 대기
                            break

                        # 패킷의 시작 바이트 찾기
                        start_index = buffer.find(b'\xf2')
                        if start_index == -1:
                            # 시작 바이트가 없으면 버퍼 초기화
                            buffer = bytearray()
                            break
                        elif len(buffer) - start_index < 19:
                            # 전체 패킷을 아직 받지 못함
                            break
                        else:
                            # 패킷 추출
                            packet = buffer[start_index:start_index + 19]
                            buffer = buffer[start_index + 19:]

                            # 패킷 파싱
                            self.parse_packet(packet)

            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                break

    def parse_packet(self, packet):
        if packet[0] != 0xf2:
            self.get_logger().error(f"Invalid packet start byte: {packet[0]}")
            return

        try:
            # 나머지 18바이트를 9개의 int16_t로 파싱 (리틀엔디안 가정)
            # 자이로 x, y, z; 가속도 x, y, z; 지자계 x, y, z
            data = struct.unpack('<hhhhhhhhh', packet[1:19])
            gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z = data

            # IMU 메시지 생성
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"  # 실제 IMU 프레임 ID로 수정

            # Orientation 설정 (쿼터니언)
            # 쿼터니언 데이터를 제공받지 않으므로, 단위 쿼터니언 사용
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0

            # Angular velocity (자이로스코프 데이터)
            # 단위: rad/s (예: 도 단위를 rad로 변환)
            imu_msg.angular_velocity.x = gyro_x * (math.pi / 180.0)
            imu_msg.angular_velocity.y = gyro_y * (math.pi / 180.0)
            imu_msg.angular_velocity.z = gyro_z * (math.pi / 180.0)

            # Linear acceleration (가속도계 데이터)
            # 단위: m/s² (예: 센서 스케일에 맞게 조정)
            imu_msg.linear_acceleration.x = accel_x * 0.1  # 예시 스케일링 값
            imu_msg.linear_acceleration.y = accel_y * 0.1
            imu_msg.linear_acceleration.z = accel_z * 0.1

            # Covariance 설정 (신뢰도에 따라 조정)
            imu_msg.orientation_covariance = [0.0] * 9
            imu_msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            imu_msg.linear_acceleration_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1
            ]

            # IMU 메시지 퍼블리시
            self.imu_publisher.publish(imu_msg)

            # 디버그 로그
            self.get_logger().debug(
                f"IMU Data - Gyro: ({gyro_x}, {gyro_y}, {gyro_z}), "
                f"Accel: ({accel_x}, {accel_y}, {accel_z}), "
                f"Mag: ({mag_x}, {mag_y}, {mag_z})"
            )

        except struct.error as e:
            self.get_logger().error(f"Packet parsing error: {e}")

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    agv_controller = AGVController()
    rclpy.spin(agv_controller)
    agv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
