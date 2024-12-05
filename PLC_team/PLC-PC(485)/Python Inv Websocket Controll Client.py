import asyncio
import websockets

async def send_signal():
    uri = "ws://172.30.1.31:8765"  # WebSocket 서버 주소 (서버의 IP로 변경)

    # WebSocket 서버에 연결
    async with websockets.connect(uri) as websocket:
        while True:
            # 사용자 입력 받기 (방향 명령)
            direction = input("방향을 입력하세요 (0: 정지, 1: 정방향, 2: 역방향, q: 종료): ").strip().lower()

            if direction == 'q':
                # 'q' 입력 시 프로그램 종료
                await websocket.send(direction)
                print("프로그램 종료.")
                break

            # 방향 명령을 서버로 전송
            await websocket.send(direction)
            response = await websocket.recv()
            print(f"서버 응답: {response}")

# WebSocket 클라이언트 실행
if __name__ == "__main__":
    asyncio.run(send_signal())
