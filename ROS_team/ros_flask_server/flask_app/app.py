from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO

from websocket import create_connection  # WebSocket 클라이언트 추가
import json  # JSON 모듈 추가

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

app = Flask(__name__)
socketio = SocketIO(app)

# 전역 변수로 최신 상태 저장
robot_arm_status = "No Data"
agv_status = "No Data"

class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('flask_ros2_publisher')
        self.status_publishers = {
            'robot_arm_status': self.create_publisher(String, '/robot_arm_status', 10),
            'agv_status': self.create_publisher(String, '/agv_status', 10)
        }
        self.command_publishers = {
            'robot_arm_command': self.create_publisher(String, '/robot_arm_command', 10),
            'agv_command': self.create_publisher(String, '/agv_command', 10)
        }
    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {command}")

    def publish_status(self, topic, message):
        if topic in self.status_publishers:
            msg = String()
            msg.data = message
            self.status_publishers[topic].publish(msg)
            self.get_logger().info(f"Published to {topic}: {message}")

    def publish_command(self, topic, message):
        if topic in self.command_publishers:
            msg = String()
            msg.data = message
            self.command_publishers[topic].publish(msg)
            self.get_logger().info(f"Published to {topic}: {message}")

# ROS 2 노드 초기화
def init_ros2_node():
    rclpy.init()
    global ros2_node
    ros2_node = ROS2Publisher()


        # 초기 메시지 퍼블리시
    # 서버 시작 시 초기 메시지 퍼블리시
    ros2_node.publish_status('robot_arm_status', "No Data")
    ros2_node.publish_status('agv_status', "No Data")
    ros2_node.publish_command('robot_arm_command', "idle")
    ros2_node.publish_command('agv_command', "idle")
    rclpy.spin(ros2_node)

# ROS 2 노드를 별도 스레드에서 실행
ros2_thread = threading.Thread(target=init_ros2_node, daemon=True)
ros2_thread.start()


# WebSocket 연결 설정
ROS_BRIDGE_URL = "ws://192.168.0.200:9090"
ros_bridge_ws = None


def init_ros_bridge():
    global ros_bridge_ws
    try:
        ros_bridge_ws = create_connection(ROS_BRIDGE_URL)
        app.logger.info(f"Connected to ROS Bridge at {ROS_BRIDGE_URL}")
    except Exception as e:
        app.logger.error(f"Failed to connect to ROS Bridge: {e}")


@app.route("/update_arm", methods=["POST"])
def update_status():
    global robot_arm_status
    data = request.get_json()
    robot_arm_status = data.get("data", "No Data")
    socketio.emit("robot_arm_update", {"status": robot_arm_status})  # 클라이언트에 상태 전송
    return jsonify({"status": "success"}), 200

@app.route("/update_agv", methods=["POST"])
def update_agv_status():
    global agv_status
    data = request.get_json()
    agv_status = data.get("data", "No Data")
    socketio.emit("agv_update", {"status": agv_status})  # 클라이언트에 상태 전송
    return jsonify({"status": "success"}), 200


@app.route("/send_command", methods=["POST"])
def send_command():
    try:
        data = request.get_json()
        topic = data.get("topic")
        command = data.get("command")
        if topic in ["robot_arm_command", "agv_command"] and command in ["start", "stop"]:
            # WebSocket으로 ROS 브리지에 명령 전송
            message = {
                "op": "publish",
                "topic": f"/{topic}",
                "msg": {"data": command}
            }
            ros_bridge_ws.send(json.dumps(message))
            app.logger.info(f"Published to {topic}: {command}")
            return jsonify({"status": "success", "command": command}), 200
        else:
            return jsonify({"error": "Invalid topic or command"}), 400
    except Exception as e:
        app.logger.error(f"Error sending command: {e}")
        return jsonify({"error": str(e)}), 500


@app.route("/")
def index():
    # 상태 정보를 HTML 템플릿에 전달
    return render_template("index.html", 
                           robot_arm_status=robot_arm_status, 
                           agv_status=agv_status)

if __name__ == "__main__": 
    # WebSocket 연결 초기화
    threading.Thread(target=init_ros_bridge, daemon=True).start()
    socketio.run(app, host="0.0.0.0", port=5000)