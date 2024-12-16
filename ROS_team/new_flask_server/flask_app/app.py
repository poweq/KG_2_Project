from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO

from websocket import create_connection  # WebSocket 클라이언트 추가
import json  # JSON 모듈 추가

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

app = Flask(__name__, static_folder='static', template_folder='templates')
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
            'agv_command': self.create_publisher(String, '/agv_command', 10),
            'pusher_command': self.create_publisher(String, '/pusher_command', 10),
            'case_command': self.create_publisher(String, '/case_command', 10)
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
    ros2_node.publish_command('pusher_status', "No Data")

    ros2_node.publish_command('robot_arm_command', "idle")
    ros2_node.publish_command('agv_command', "idle")
    ros2_node.publish_command('pusher_command', "idle")
    ros2_node.publish_command('case_command', "idle")
    
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

# page render
@app.route("/case_page_a")
def case_page_a():
    # Case A 페이지 렌더링
    return render_template("case_page_a.html", case="A")


@app.route("/case_page_b")
def case_page_b():
    # Case B 페이지 렌더링
    return render_template("case_page_b.html", case="B")


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
    agv_status = data.get("data", "wait")
    socketio.emit("agv_update", {"status": agv_status})  # 클라이언트에 상태 전송
    return jsonify({"status": "success"}), 200

#pusher
@app.route("/update_pusher_x", methods=["POST"])
def update_pusher_x():
    try:
        data = request.get_json()
        value = data.get("value")
        if isinstance(value, int) and value > 0:  # 자연수 확인
            socketio.emit("pusher_x_update", {"value": value})  # 클라이언트에 상태 전송
            app.logger.info(f"Updated Pusher X with value: {value}")
            return jsonify({"status": "success", "value": value}), 200
        else:
            return jsonify({"error": "Invalid value"}), 400
    except Exception as e:
        app.logger.error(f"Error updating pusher X: {e}")
        return jsonify({"error": str(e)}), 500


@app.route("/update_pusher_y", methods=["POST"])
def update_pusher_y():
    try:
        data = request.get_json()
        value = data.get("value")
        if isinstance(value, int) and value > 0:  # 자연수 확인
            socketio.emit("pusher_y_update", {"value": value})  # 클라이언트에 상태 전송
            app.logger.info(f"Updated Pusher Y with value: {value}")
            return jsonify({"status": "success", "value": value}), 200
        else:
            return jsonify({"error": "Invalid value"}), 400
    except Exception as e:
        app.logger.error(f"Error updating pusher Y: {e}")
        return jsonify({"error": str(e)}), 500
    
#case
@app.route("/update_case_a", methods=["POST"])
def update_case_a():
    try:
        data = request.get_json()
        value = data.get("value", "No Data")
        socketio.emit("case_a_update", {"value": value})  # 소켓 이벤트 전송
        app.logger.info(f"Case A Data Updated: {value}")
        return jsonify({"status": "success"}), 200
    except Exception as e:
        app.logger.error(f"Error updating Case A: {e}")
        return jsonify({"error": str(e)}), 500

#case
@app.route("/update_case_b", methods=["POST"])
def update_case_b():
    try:
        data = request.get_json()
        value = data.get("value", "No Data")
        socketio.emit("case_b_update", {"value": value})  # 소켓 이벤트 전송
        app.logger.info(f"Case B Data Updated: {value}")
        return jsonify({"status": "success"}), 200
    except Exception as e:
        app.logger.error(f"Error updating Case B: {e}")
        return jsonify({"error": str(e)}), 500


@app.route("/send_command", methods=["POST"])
def send_command():
    try:
        data = request.get_json()
        topic = data.get("topic")
        command = data.get("command")
        app.logger.info(f"command to {topic}: {command}")
        
        # 로봇암과 AGV 명령 처리 
        if topic in ["robot_arm_command", "agv_command"]:
            app.logger.info(f"topic command")
            if command in ["start", "stop"]:
                # ROS 2 퍼블리시 로직
                message = {"op": "publish", "topic": f"/{topic}", "msg": {"data": command}}
                ros_bridge_ws.send(json.dumps(message))
                app.logger.info(f"Published to {topic}: {command}")
                return jsonify({"status": "success", "command": command}), 200
            else:
                return jsonify({"error": "Invalid command for topic"}), 400
        
        # 푸셔 명령 처리    
        elif topic == "pusher_command":
            if ":" in command:  # x:n 또는 y:n 형식 확인
                message = {"op": "publish", "topic": f"/{topic}", "msg": {"data": command}}
                ros_bridge_ws.send(json.dumps(message))
                app.logger.info(f"Published to {topic}: {command}")
                return jsonify({"status": "success", "command": command}), 200
            else:
                return jsonify({"error": "Invalid pusher command"}), 400
        #case  
        elif topic == "case_command":
            if ":" in command:  # a:n 또는 b:open 형식 확인
                case, action = command.split(":", 1)  # "a" 또는 "b", "1" 또는 "open"으로 분리
                if case in ["a", "b"] and (action.isdigit() or action in ["open", "close"]):
                    # 메시지 생성 및 ROS 브리지로 전송
                    message = {"op": "publish", "topic": f"/{topic}", "msg": {"data": command}}
                    ros_bridge_ws.send(json.dumps(message))
                    app.logger.info(f"Published to {topic}: {command}")
                    return jsonify({"status": "success", "command": command}), 200
                else:
                    # 유효하지 않은 명령 처리
                    app.logger.warning(f"Invalid case command: {command}")
                    return jsonify({"error": "Invalid case command"}), 400
            else:
                # 형식 오류 처리
                app.logger.warning(f"Invalid command format: {command}")
                return jsonify({"error": "Invalid command format"}), 400

            
        else:
            return jsonify({"error": "Unknown topic"}), 400
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