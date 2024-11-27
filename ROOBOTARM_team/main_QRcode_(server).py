# server_url = "ws://192.168.0.200:9090"
# publish_message = { "op": "publish", "topic": "/robot_arm", "msg": { "data": "Hello from robot arm" } }

import socket

# 서버 설정
server_ip = "192.168.0.182"  # 클라이언트(로봇)의 IP 주소로 변경
server_port = 7000  # 클라이언트와 동일한 포트 번호

# UDP 소켓 생성
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"신호 송신 서버 시작! 로봇 IP: {server_ip}, 포트: {server_port}")
print("1: 실행, 0: 대기, r: 리셋, q: 종료")

try:
    while True:
        # 사용자 입력 받기
        user_input = input("신호 입력 (1, 0, r, q): ").strip()
        if user_input not in ["1", "0", "r", "q"]:
            print("잘못된 입력입니다. 1, 0, r, 또는 q를 입력하세요.")
            continue

        # 신호를 클라이언트로 전송
        server_socket.sendto(user_input.encode(), (server_ip, server_port))
        print(f"신호 '{user_input}'를 클라이언트로 전송했습니다.")
        
        if user_input == "q":
            print("서버를 종료합니다.")
            break
finally:
    server_socket.close()
