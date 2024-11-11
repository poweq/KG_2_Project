import serial
import time

# USB-Serial 포트 설정
port = 'COM4'  # 사용 중인 포트로 수정
baudrate = 9600  # PLC에 맞는 통신 속도 설정
timeout = 1  # 읽기 타임아웃 설정

# 시리얼 포트 열기
ser = serial.Serial(port, baudrate, timeout=timeout)

# M100 비트를 HIGH로 설정하기 위한 명령어 예시
# 명령어 형식은 PLC의 매뉴얼에 따라 수정해야 합니다.
command = bytes.fromhex('41 42 43 44 45 46')  # 예시: 실제 명령어로 수정 필요

try:
    # 명령 송출
    ser.write(command)
    print(f"Sent command to set M100 HIGH")

    # 응답 수신 (필요시)
    response = ser.read(2)  # 응답 크기 조정
    if response:
        print(f"Received: {response.hex()}")
    else:
        print("No response received.")

except KeyboardInterrupt:
    print("송출 중단")
finally:
    ser.close()
