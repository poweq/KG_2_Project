import time
import websocket

def create_connection():
    while True:
        try:
            ws = websocket.WebSocket()
            ws.connect("ws://172.30.1.61")
            print("Connected to WebSocket server")
            return ws
        except Exception as e:
            print(f"연결 실패: {e}, 5초 후 재시도")
            time.sleep(5)  # 5초 후 재시도

# WebSocket 서버와 연결
ws = create_connection()

while True:
    # 사용자 입력을 받아서 서버로 전송
    user_input = input("Say something (type 'q' to quit): ")
    if user_input.lower() == 'q':
        print("Exiting...")
        break

    try:
        ws.send(user_input)

        # 서버 응답을 기다리고 출력
        result = ws.recv()
        print("Received: " + result)
    except Exception as e:
        print(f"오류 발생: {e}")
        print("연결이 끊어졌습니다. 재연결 시도 중...")
        ws.close()
        ws = create_connection()  # 연결 재시도

# WebSocket 연결 종료
ws.close()
