import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class AGVController(Node):
    def __init__(self):
        super().__init__('agv_controller_node')
        
        # 시리얼 포트 설정 (Raspberry Pi의 ttyAMA1 포트를 사용)
        try:
            self.serial_port = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
            self.get_logger().info("Serial connection established on /dev/ttyAMA1")
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

        # linear.x 값으로 모터 속도 설정
        linear_x = msg.linear.x
        motor_speed = 0 if linear_x <= 0 else min(int(linear_x * 50), 50)

        # 모터 제어 명령 생성 및 전송
        command = f"a:{motor_speed}\nb:{motor_speed}\nc:{motor_speed}\nd:{motor_speed}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent motor speed: {motor_speed}")

def main(args=None):
    rclpy.init(args=args)
    agv_controller = AGVController()
    rclpy.spin(agv_controller)
    agv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
