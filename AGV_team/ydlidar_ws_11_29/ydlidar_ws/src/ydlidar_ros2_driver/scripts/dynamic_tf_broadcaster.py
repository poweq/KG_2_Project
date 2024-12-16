#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 초기 Map -> Odom 값
        self.map_to_odom = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

        # 초기 Odom -> Base Footprint 값
        self.odom_to_base_footprint = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

        # /odom 토픽 구독
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10  # QoS depth
        )

        # 주기적으로 TF 브로드캐스트
        self.timer = self.create_timer(0.1, self.broadcast_transforms)  # 10Hz

    def odom_callback(self, msg):
        """
        /odom 토픽에서 데이터를 받아 Odom -> Base Footprint 값 업데이트
        """
        self.odom_to_base_footprint['x'] = msg.pose.pose.position.x
        self.odom_to_base_footprint['y'] = msg.pose.pose.position.y
        self.odom_to_base_footprint['z'] = msg.pose.pose.position.z

        # Quaternion -> Yaw 변환
        quaternion = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        self.odom_to_base_footprint['yaw'] = yaw

    def broadcast_transforms(self):
        # Map -> Odom (고정값 또는 다른 노드에서 동적 데이터를 받을 수 있음)
        self.broadcast_transform(
            parent_frame='map',
            child_frame='odom',
            transform=self.map_to_odom
        )

        # Odom -> Base Footprint (동적으로 업데이트된 값 사용)
        self.broadcast_transform(
            parent_frame='odom',
            child_frame='base_footprint',
            transform=self.odom_to_base_footprint
        )

    def broadcast_transform(self, parent_frame, child_frame, transform):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = transform['x']
        t.transform.translation.y = transform['y']
        t.transform.translation.z = transform['z']

        # Yaw 값만 사용하여 Quaternion 생성
        quaternion = self.yaw_to_quaternion(transform['yaw'])
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def yaw_to_quaternion(yaw):
        import math
        # Roll과 Pitch는 0으로 고정
        roll = 0.0
        pitch = 0.0

        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        import math
        # Quaternion -> Yaw 변환
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
