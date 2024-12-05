from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from rosbridge_client import ROS2WebSocketClient
import threading

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")  # CORS 허용 설정

# ROS2 웹소켓 클라이언트 초기화
ros_client = ROS2WebSocketClient(rosbridge_ip='192.168.0.200', rosbridge_port=9090)

def ros_subscriber():
    # 로봇 암 상태 콜백 함수
    def robot_arm_status_callback(msg):
        status = msg.get('data')
        print(f"Received robot arm status: {status}")
        socketio.emit('robot_arm_status', {'status': status})

    # AGV 상태 콜백 함수
    def agv_status_callback(msg):
        status = msg.get('data')
        print(f"Received AGV status: {status}")
        socketio.emit('agv_status', {'status': status})

    # 토픽 구독 설정
    ros_client.receive_data('/robot_arm_status', robot_arm_status_callback, msg_type='std_msgs/String')
    ros_client.receive_data('/agv_status', agv_status_callback, msg_type='std_msgs/String')

# 백그라운드에서 ROS2 웹소켓 클라이언트 실행
def start_ros_client():
    ros_client.connect()
    ros_subscriber()

ros_thread = threading.Thread(target=start_ros_client)
ros_thread.daemon = True
ros_thread.start()

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('robot_arm_command')
def handle_robot_arm_command(json):
    command = json.get('command')
    print(f"Received command from client: {command}")
    ros_client.send_data('/robot_arm_command', command, msg_type='std_msgs/String')

@socketio.on('agv_command')
def handle_agv_command(json):
    command = json.get('command')
    print(f"Received command from client: {command}")
    ros_client.send_data('/agv_command', command, msg_type='std_msgs/String')

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
