from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO
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
        self.publisher = self.create_publisher(String, '/robot_arm_command', 10)
    
    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {command}")

# ROS 2 노드 초기화
def init_ros2_node():
    rclpy.init()
    global ros2_node
    ros2_node = ROS2Publisher()
    rclpy.spin(ros2_node)

# ROS 2 노드를 별도 스레드에서 실행
ros2_thread = threading.Thread(target=init_ros2_node, daemon=True)
ros2_thread.start()

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
        command = data.get("command")
        if command in ["start", "stop"]:
            ros2_node.publish_command(command)
            return jsonify({"status": "success", "command": command}), 200
        else:
            return jsonify({"error": "Invalid command"}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/")
def index():
    # 상태 정보를 HTML 템플릿에 전달
    return render_template("index.html", 
                           robot_arm_status=robot_arm_status, 
                           agv_status=agv_status)

if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000)
