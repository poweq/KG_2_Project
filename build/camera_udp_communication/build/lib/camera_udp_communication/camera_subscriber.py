import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import numpy as np

class UDPImageReceiver(Node):
    def __init__(self, udp_ip='192.168.0.103', udp_port=5005):
        super().__init__('udp_image_receiver')
        self.publisher_ = self.create_publisher(Image, 'camera_image_udp', 10)
        self.bridge = CvBridge()
        
        # UDP 소켓 설정
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        # 데이터 수신용 버퍼 초기화
        self.buffer = bytearray()

        # 타이머 설정 (수신 주기 조절)
        self.timer = self.create_timer(1/30, self.receive_and_publish)

    def receive_and_publish(self):
        try:
            data, _ = self.sock.recvfrom(65536)
            self.buffer.extend(data)
            
            # 충분한 데이터가 수신되었으면 이미지로 변환
            if len(self.buffer) > 0:
                nparr = np.frombuffer(self.buffer, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if frame is not None:
                    # OpenCV 이미지 -> ROS Image 메시지 변환
                    image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    self.publisher_.publish(image_msg)
                    self.get_logger().info("이미지 수신 및 퍼블리시 완료")
                    self.buffer.clear()  # 버퍼 초기화
        except Exception as e:
            self.get_logger().info(f"데이터 수신 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UDPImageReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
