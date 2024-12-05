import websocket
import json
import threading

class ROS2WebSocketClient:
    def __init__(self, rosbridge_ip='192.168.0.200', rosbridge_port=9090):
        self.rosbridge_ip = rosbridge_ip
        self.rosbridge_port = rosbridge_port
        self.ws = None
        self.topics = {}
        self.advertised_topics = set()
        self.is_connected = False

    def connect(self):
        websocket.enableTrace(False)
        ws_url = f"ws://{self.rosbridge_ip}:{self.rosbridge_port}"
        self.ws = websocket.WebSocketApp(
            ws_url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        # 별도의 스레드에서 웹소켓 실행
        self.thread = threading.Thread(target=self.ws.run_forever)
        self.thread.daemon = True
        self.thread.start()
        # 연결 완료될 때까지 대기
        while not self.is_connected:
            pass

    def on_open(self, ws):
        print("Connected to ROS 2 bridge server.")
        self.is_connected = True

    def on_message(self, ws, message):
        data = json.loads(message)
        if data.get('op') == 'publish':
            topic = data.get('topic')
            msg = data.get('msg')
            if topic in self.topics:
                callback = self.topics[topic]
                callback(msg)

    def on_error(self, ws, error):
        print("WebSocket error:", error)

    def on_close(self, ws, close_status_code, close_msg):
        print("Disconnected from ROS 2 bridge server.")
        self.is_connected = False

    def advertise_topic(self, topic, msg_type):
        """토픽을 광고하여 메시지 타입을 지정"""
        if topic not in self.advertised_topics:
            advertise_message = {
                "op": "advertise",
                "topic": topic,
                "type": msg_type
            }
            self.ws.send(json.dumps(advertise_message))
            self.advertised_topics.add(topic)
            print(f"Advertised topic: {topic} with type: {msg_type}")

    def send_data(self, topic, message, msg_type='std_msgs/String'):
        """ROS 2 토픽에 데이터 퍼블리시"""
        # 토픽이 광고되지 않았으면 광고
        if topic not in self.advertised_topics:
            self.advertise_topic(topic, msg_type)

        publish_message = {
            "op": "publish",
            "topic": topic,
            "msg": {
                "data": message
            }
        }
        self.ws.send(json.dumps(publish_message))

    def receive_data(self, topic, callback, msg_type='std_msgs/String'):
        """ROS 2 토픽을 구독하고 수신한 데이터를 콜백 함수로 처리"""
        self.topics[topic] = callback
        subscribe_message = {
            "op": "subscribe",
            "topic": topic,
            "type": msg_type
        }
        self.ws.send(json.dumps(subscribe_message))
        print(f"Subscribed to topic: {topic} with type: {msg_type}")

    def disconnect(self):
        self.ws.close()
        self.thread.join()
