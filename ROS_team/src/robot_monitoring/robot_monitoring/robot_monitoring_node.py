#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_monitoring.srv import RobotCommand  # 사용자 정의 서비스 임포트
from std_msgs.msg import String

class RobotMonitoringNode(Node):
    def __init__(self):
        super().__init__('robot_monitoring_node')

        # 서비스 서버 생성
        self.srv = self.create_service(RobotCommand, 'robot_monitoring_service', self.handle_service_request)

        # 토픽 퍼블리셔 생성
        self.robot_arm_status_pub = self.create_publisher(String, 'robot_arm_status', 10)
        self.agv_status_pub = self.create_publisher(String, 'agv_status', 10)

        # 토픽 구독자 생성
        self.robot_arm_command_sub = self.create_subscription(String, 'robot_arm_command', self.robot_arm_command_callback, 10)
        self.agv_command_sub = self.create_subscription(String, 'robot_agv_command', self.agv_command_callback, 10)

        # 내부 상태 변수
        self.robot_arm_running = False
        self.agv_running = False

        self.robot_arm_pose = 1
        self.agv_pose = 1

        # 타이머 설정
        self.timer_period = 1.0  # 1초마다 콜백 실행
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Robot Monitoring Node가 시작되었습니다.')

    def handle_service_request(self, request, response):
        command = request.command.lower()
        if command == 'start':
            self.get_logger().info('서비스 요청: START')
            response.success = True
            response.message = 'Monitoring started.'
        elif command == 'stop':
            self.get_logger().info('서비스 요청: STOP')
            # 내부 상태 초기화
            self.robot_arm_running = False
            self.agv_running = False
            self.robot_arm_pose = 1
            self.agv_pose = 1
            response.success = True
            response.message = 'Monitoring stopped.'
        else:
            self.get_logger().info(f'알 수 없는 명령: {command}')
            response.success = False
            response.message = f'Unknown command: {command}'
        return response

    def robot_arm_command_callback(self, msg):
        if msg.data.lower() == 'start':
            self.get_logger().info('로봇 암 명령 수신: START')
            self.robot_arm_running = True
        elif msg.data.lower() == 'stop':
            self.get_logger().info('로봇 암 명령 수신: STOP')
            self.robot_arm_running = False
            self.robot_arm_pose = 1
        else:
            self.get_logger().info(f'로봇 암 알 수 없는 명령: {msg.data}')

    def agv_command_callback(self, msg):
        if msg.data.lower() == 'start':
            self.get_logger().info('AGV 명령 수신: START')
            self.agv_running = True
        elif msg.data.lower() == 'stop':
            self.get_logger().info('AGV 명령 수신: STOP')
            self.agv_running = False
            self.agv_pose = 1
        else:
            self.get_logger().info(f'AGV 알 수 없는 명령: {msg.data}')

    def timer_callback(self):
        if self.robot_arm_running:
            # 로봇 암 상태 퍼블리시
            msg = String()
            msg.data = f'pose {self.robot_arm_pose}'
            self.robot_arm_status_pub.publish(msg)
            self.get_logger().info(f'로봇 암 상태 퍼블리시: {msg.data}')
            self.robot_arm_pose += 1
            if self.robot_arm_pose > 5:
                self.robot_arm_pose = 1

        if self.agv_running:
            # AGV 상태 퍼블리시
            msg = String()
            msg.data = f'pose {self.agv_pose}'
            self.agv_status_pub.publish(msg)
            self.get_logger().info(f'AGV 상태 퍼블리시: {msg.data}')
            self.agv_pose += 1
            if self.agv_pose > 5:
                self.agv_pose = 1

def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitoringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
