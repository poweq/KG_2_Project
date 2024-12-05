
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
        linear_y = msg.linear.y
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
                motor_command[1] -= reduction  # a
                motor_command[2] -= reduction  # b 감속
                motor_command[3] += boost  # c 가속
                motor_command[4] += boost  # d 가속
            elif angular_z > 0:
                motor_command[1] += boost  # a 가속
                motor_command[2] += boost  # b 가속
                motor_command[3] -= reduction  # c 감속
                motor_command[4] -= reduction  # d 감속

            # 후진 시 부호 반전
            if linear_x < 0:
                motor_command[1] = -motor_command[1]
                motor_command[2] = -motor_command[2]
                motor_command[3] = -motor_command[3]
                motor_command[4] = -motor_command[4]


            # linear_y 처리 추가
            if linear_y != 0:
                lateral_speed = max(min(int(abs(linear_y) * self.limit_speed), self.limit_speed), -self.limit_speed)
                lateral_speed = lateral_speed * 0.5
                if linear_y > 0:
                    # linear_y가 양수일 때
                    motor_command[1] = -lateral_speed  # a
                    motor_command[2] = lateral_speed  # b
                    motor_command[3] = lateral_speed  # c
                    motor_command[4] = -lateral_speed  # d
                elif linear_y < 0:
                    # linear_y가 음수일 때
                    motor_command[1] = lateral_speed  # a
                    motor_command[2] = -lateral_speed  # b
                    motor_command[3] = -lateral_speed  # c
                    motor_command[4] = lateral_speed  # d

        # 리스트 기반으로 명령 문자열 생성 및 전송
        motor_command_str = ','.join(map(str, motor_command)) + '\n'
        self.serial_port.write(motor_command_str.encode())

        # 로그에 리스트와 각 모터에 전송되는 명령을 출력
        self.get_logger().info(
            f"Linear X: {linear_x},Linear Y: {linear_y} , Angular Z: {angular_z}\n"
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
                        if len(buffer) < 20:
                            # 패킷 크기 미달 시 대기
                            break

                        # 패킷의 시작 바이트 찾기
                        start_index = buffer.find(b'\xf3')
                        if start_index == -1:
                            # 시작 바이트가 없으면 버퍼 초기화
                            buffer = bytearray()
                            break
                        elif len(buffer) - start_index < 20:
                            # 전체 패킷을 아직 받지 못함
                            break
                        else:
                            # 패킷 추출
                            packet = buffer[start_index:start_index + 20]
                            buffer = buffer[start_index + 20:]

                            # 패킷 파싱
                            self.parse_packet(packet)

            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                break

    def parse_packet(self, packet):

        try:
            # 나머지 18바이트를 9개의 int16_t로 파싱 (리틀엔디안 가정)
            # 자이로 x, y, z; 가속도 x, y, z; 지자계 x, y, z
            data = struct.unpack('<H9h', packet)


            # 센서 값 추출
            gyro_x, gyro_y, gyro_z = data[1], data[2], data[3]
            acc_x, acc_y, acc_z = data[4], data[5], data[6]
            mag_x, mag_y, mag_z = data[7], data[8], data[9]

            # 변환: 자이로스코프 (raw -> rad/s)
            gyro_scale = 131.0
            angular_velocity_x = (gyro_x / gyro_scale) * (math.pi / 180.0) *100
            angular_velocity_y = (gyro_y / gyro_scale) * (math.pi / 180.0) *100
            angular_velocity_z = (gyro_z / gyro_scale) * (math.pi / 180.0) *100

            # 변환: 가속도계 (raw -> m/s²)
            accel_scale = 16384.0
            linear_acceleration_x = (acc_x / accel_scale) * 9.8 *100
            linear_acceleration_y = (acc_y / accel_scale) * 9.8 *100
            linear_acceleration_z = (acc_z / accel_scale) * 9.8 *100

            # IMU 메시지 생성
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            imu_msg.angular_velocity.x = angular_velocity_x
            imu_msg.angular_velocity.y = angular_velocity_y
            imu_msg.angular_velocity.z = angular_velocity_z

            imu_msg.linear_acceleration.x = linear_acceleration_x
            imu_msg.linear_acceleration.y = linear_acceleration_y
            imu_msg.linear_acceleration.z = linear_acceleration_z

            # IMU 메시지 퍼블리시
            self.imu_publisher.publish(imu_msg)
            
            self.get_logger().debug(
                f"IMU Data - Gyro (raw): ({gyro_x}, {gyro_y}, {gyro_z}), "
                f"Accel (raw): ({acc_x}, {acc_y}, {acc_z}), "
                f"Mag (raw): ({mag_x}, {mag_y}, {mag_z})"
            )
            self.get_logger().debug(
                f"IMU Data - Gyro (rad/s): ({angular_velocity_x:.3f}, {angular_velocity_y:.3f}, {angular_velocity_z:.3f}), "
                f"Accel (m/s²): ({linear_acceleration_x:.3f}, {linear_acceleration_y:.3f}, {linear_acceleration_z:.3f})"
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

