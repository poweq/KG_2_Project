import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import serial
import math
import time

from std_msgs.msg import String, Int64, Int8
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class MyCar(object):
    def __init__(self, port='/dev/ttyAMA2'):
        self.port = port
        self.ser = serial.Serial(port, 115200)
        self.header = [0xfe, 0xfe]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        self.lastTime = Clock().now()

    def restore_run(self):
        res = 0
        print("if you want restore run, please input 1, then press enter")

        while res != 1:
            res = int(input())  # 사용자의 입력을 정수로 변환하여 저장
        self.restore()  # restore 함수 호출

    def restore(self):
        cmd = bytearray([0xfe, 0xfe, 0x01, 0x00, 0x01, 0x02])
        self.ser.write(cmd)

    def readSpeed(self):
        buf_header = bytearray(1)
        buf = bytearray(16)
        header_found = False
        count = 0

        while not header_found:
            count += 1
            ret = self.ser.readinto(buf_header)
            if ret != 1:
                continue
            if buf_header[0] != self.header[0]:
                continue
            header_2_found = False
            while not header_2_found:
                ret = self.ser.readinto(buf_header)
                if ret != 1:
                    continue
                if buf_header[0] != self.header[0]:
                    continue
                header_2_found = True
            header_found = True

        ret = self.ser.readinto(buf)
        if ret != 16:
            print("Read error")
            return False

        index = 0
        self.vx = (buf[index] - 128.0) * 0.01
        self.vy = (buf[index + 1] - 128.0) * 0.01
        self.vtheta = (buf[index + 2] - 128.0) * 0.01

        self.ax = ((buf[index + 3] + buf[index + 4] * 256) - 10000) * 0.001
        self.ay = ((buf[index + 5] + buf[index + 6] * 256) - 10000) * 0.001
        self.az = ((buf[index + 7] + buf[index + 8] * 256) - 10000) * 0.001

        self.wx = ((buf[index + 9] + buf[index + 10] * 256) - 10000) * 0.1
        self.wy = ((buf[index + 11] + buf[index + 12] * 256) - 10000) * 0.1
        self.wz = ((buf[index + 13] + buf[index + 14] * 256) - 10000) * 0.1

        current_time = Clock().now()
        dt_nano = (current_time - self.lastTime).nanoseconds
        dt = dt_nano / 1e9

        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_th = self.vtheta * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th
        self.lastTime = current_time
        return True


    def write_speed(self, movex, movey, rot):
        if movex == 10 and movey == 10 and rot == 10:
            buf = [0xfe, 0xfe, 0x01, 0x01, 0x01, 0x03]
            self.ser.write(bytearray(buf))
            buf_header = bytearray(1)

            header_found = False
            start_time = time.time()

            while True:
                ret = self.ser.readinto(buf_header)
                if ret != 1:
                    continue
                if buf_header[0] != self.header[0]:
                    continue

                header_2_found = False
                while not header_2_found:
                    ret = self.ser.readinto(buf_header)
                    if ret != 1:
                        continue
                    if buf_header[0] != self.header[0]:
                        continue
                    header_2_found = True

                header_found = True
                ret = self.ser.readinto(buf_header)
                if buf_header[0] == 0x01:
                    ret = self.ser.readinto(buf_header)
                    if buf_header[0] == 0x01:
                        ret = self.ser.readinto(buf_header)
                        msg = Int8()
                        msg.data = int(buf_header[0]) // 10
                        print(f"Voltage: {msg.data}")
                        break

                if time.time() - start_time > 3:
                    print("Get Voltage timeout")
                    break

        else:
            movex = max(min(movex, 1.0), -1.0)
            movey = max(min(movey, 1.0), -1.0)
            rot = max(min(rot, 1.0), -1.0)

            x_send = int(movex * 10) + 128
            y_send = int(movey * 10) + 128
            rot_send = int(rot * 10) + 128
            check = (x_send + y_send + rot_send) % 256
            buf = bytearray([self.header[0], self.header[1], x_send, y_send, rot_send, check])
            print(x_send, y_send, rot_send)
            self.ser.write(buf)

    def move_horizontal(self, direction, speed=0.5):
        if direction == 'right':
            self.write_speed(0, speed, 0)
        elif direction == 'left':
            self.write_speed(0, -speed, 0)
        else:
            print("Invalid direction. Use 'left' or 'right'")

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.linear_subscriber = self.create_subscription(Int64, 'linear', self.linear_callback, 100)
        self.angle_subscriber = self.create_subscription(Int64, 'angular', self.angle_callback, 10)
        self.exe_timer = self.create_timer(0.05, self.execute)
        self.mycar = MyCar('/dev/ttyAMA2')
        self.linearx = 0.0
        self.lineary = 0.0
        self.angularz = 0.0
        self.odom_broadcaster = TransformBroadcaster(self)

    def execute(self):
        self.mycar.readSpeed()
        self.mycar.write_speed(self.linearx, self.lineary, self.angularz)

        current_time = self.get_clock().now().to_msg()

        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_footprint'

        odom_quat = Quaternion()
        odom_quat.x = 0.0
        odom_quat.y = 0.0
        odom_quat.z = 0.0
        odom_quat.w = 0.0

        odom_trans.transform.translation.x = self.mycar.x
        odom_trans.transform.translation.y = self.mycar.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat

        self.odom_broadcaster.sendTransform(odom_trans)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.pose.pose.position.x = self.mycar.x
        odom_msg.pose.pose.position.y = self.mycar.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = odom_quat

        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.twist.twist.linear.x = self.mycar.vx
        odom_msg.twist.twist.linear.y = self.mycar.vy
        odom_msg.twist.twist.angular.z = self.mycar.vtheta

        self.pub_odom.publish(odom_msg)

    def linear_callback(self, msg):
        self.get_logger().info('linear X "%s"' % msg.data)
        if msg.data > 0:
            self.get_logger().info('MyAGV go forward')
        else:
            self.get_logger().info('MyAGV stop')

    def angle_callback(self, msg):
        self.get_logger().info('angle Z "%s"' % msg.data)
        if msg.data > 0:
            self.get_logger().info('MyAGV turn left')
        else:
            self.get_logger().info('MyAGV turn right')


    def cmd_callback(self, msg):
        self.get_logger().info('linear X "%s"' % msg.linear.x)
        self.linearx = msg.linear.x
        self.lineary = msg.linear.y
        self.angularz = msg.angular.z
        print(self.linearx, self.lineary, self.angularz)

        if self.lineary > 0:
            self.mycar.move_horizontal('right', self.lineary)
        elif self.lineary < 0:
            self.mycar.move_horizontal('left', abs(self.lineary))


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.get_logger().info("finished...")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
