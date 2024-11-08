import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

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

    def cmd_vel_callback(self, msg):
        if not self.serial_port:
            self.get_logger().error("Serial port not available.")
            return

        # linear.x 값과 gyro_z 값을 추출
        linear_x = msg.linear.x
        gyro_z = msg.angular.z

        # 모터 속도 리스트 초기화
        motor_command = []

        # 분기점 설정
        if linear_x == 0 and gyro_z != 0:
            # 제자리 회전 모드
            rotation_speed = max(min(int(abs(gyro_z) * self.limit_speed), self.limit_speed), -self.limit_speed)

            if gyro_z < 0:
                # 왼쪽 회전, ccw 방향
                motor_command = [0xf1, 'cc', rotation_speed]
            elif gyro_z > 0:
                # 오른쪽 회전, cw 방향
                motor_command = [0xf1, 'cw', rotation_speed]
        else:
            # 기존 동작 모드 (linear.x와 gyro_z 값을 모두 고려)
            base_speed = max(min(int(abs(linear_x) * self.limit_speed), self.limit_speed), -self.limit_speed)
            motor_command = [0xf0, base_speed, base_speed, base_speed, base_speed]

            # 감속 및 가속 비율 적용
            reduction = int(abs(base_speed) * 0.55)  # 55% 감속
            boost = int(abs(base_speed) * 0.15)  # 15% 가속

            # gyro_z에 따라 감속 및 가속 조정
            if gyro_z < 0:
                motor_command[1] -= reduction  # c 감속
                motor_command[2] -= reduction  # d 감속
                motor_command[3] += boost  # a 가속
                motor_command[4] += boost  # b 가속
            elif gyro_z > 0:
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
            f"Linear X: {linear_x}, Gyro Z: {gyro_z}\n"
            f"Adjusted Motor Command List: {motor_command}\n"
            f"Commands sent to motors:\n"
            f"Motor Command: {motor_command}"
        )

def main(args=None):
    rclpy.init(args=args)
    agv_controller = AGVController()
    rclpy.spin(agv_controller)
    agv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
