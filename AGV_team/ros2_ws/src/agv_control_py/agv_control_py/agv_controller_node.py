
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial
import threading
import math
from rclpy.qos import qos_profile_sensor_data
from collections import deque

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

        # 자력계 보정 파라미터
        self.mag_bias = [-339.590589, 96.630322, -397.338474]  # 바이어스
        self.mag_scale = [  # 스케일 행렬 (A^-1)
            [0.122757, 0.005258, -0.009477],
            [0.005258, 0.120840, -0.005066],
            [-0.009477, -0.005066, 0.117316]
        ]

        # Yaw 값 필터링을 위한 큐
        self.N = 100  # 이동 평균을 계산할 데이터 수
        self.yaw_values = deque(maxlen=self.N)

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
            f"Linear X: {linear_x}, Linear Y: {linear_y}, Angular Z: {angular_z}\n"
            f"Adjusted Motor Command List: {motor_command}\n"
            f"Commands sent to motors: {motor_command}"
        )

    def read_serial_data(self):
        while self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    data = line.split(',')
                    if data[0] == '243' and len(data) == 10:
                        acc_x = float(data[1])
                        acc_y = float(data[2])
                        acc_z = float(data[3])
                        gyro_x = float(data[4])
                        gyro_y = float(data[5])
                        gyro_z = float(data[6])
                        mag_x = float(data[7])
                        mag_y = float(data[8])
                        mag_z = float(data[9])

                        # 자력계 보정 및 Yaw 계산
                        raw_mag = [mag_x, mag_y, mag_z]
                        calib_mag = self.apply_mag_calibration(raw_mag)
                        yaw = self.calculate_yaw(calib_mag[0], calib_mag[1])

                        # 이동 평균 필터 적용
                        self.yaw_values.append(yaw)
                        filtered_yaw = sum(self.yaw_values) / len(self.yaw_values)

                        # IMU 메시지 생성
                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = "imu_link"

                        # 가속도계 데이터 설정 (g 단위 -> m/s²)
                        imu_msg.linear_acceleration.x = acc_x * 9.80665
                        imu_msg.linear_acceleration.y = acc_y * 9.80665
                        imu_msg.linear_acceleration.z = acc_z * 9.80665

                        # 자이로스코프 데이터 설정 (°/s 단위 -> rad/s)
                        imu_msg.angular_velocity.x = math.radians(gyro_x)
                        imu_msg.angular_velocity.y = math.radians(gyro_y)
                        imu_msg.angular_velocity.z = math.radians(gyro_z)

                        # 방향 정보는 자력계로부터 계산된 Yaw 값을 사용
                        # (필요에 따라 Orientation 값을 설정하거나, 다른 패키지를 사용하여 자세 추정을 할 수 있습니다.)

                        # IMU 메시지 퍼블리시
                        self.imu_publisher.publish(imu_msg)

                        # 로그 출력 (필요 시)
                        self.get_logger().debug(
                            f"IMU Data - Accel (g): ({acc_x}, {acc_y}, {acc_z}), "
                            f"Gyro (°/s): ({gyro_x}, {gyro_y}, {gyro_z}), "
                            f"Mag (μT): ({mag_x}, {mag_y}, {mag_z})"
                        )
                        self.get_logger().debug(
                            f"Yaw: {yaw:.2f} degrees, Filtered Yaw: {filtered_yaw:.2f} degrees"
                        )

                    else:
                        self.get_logger().warning(f"Invalid data: {line}")
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def apply_mag_calibration(self, raw_mag):
        # 바이어스 제거
        temp = [raw_mag[i] - self.mag_bias[i] for i in range(3)]
        # 스케일 행렬 적용
        calib_mag = [0, 0, 0]
        for i in range(3):
            calib_mag[i] = sum(self.mag_scale[i][j] * temp[j] for j in range(3))
        return calib_mag

    def calculate_yaw(self, mag_x, mag_y):
        return math.atan2(mag_y, mag_x) * (180.0 / math.pi)

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
