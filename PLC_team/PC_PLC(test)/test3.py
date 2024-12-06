import serial

def send_ascii_command(ser, frame):
    try:
        # 명령 전송 (ASCII 형식)
        ser.write(frame.encode())  # ASCII 형식으로 전송
        print(f"보낸 프레임 (ASCII): {frame}")

        # 응답 받기 (ASCII 형식으로 응답을 읽음)
        response = ser.read(1024)  # 충분한 길이로 읽기

        if len(response) > 0:
            print(f"받은 응답 (ASCII): {response.decode('ascii', errors='ignore')}")

            # 응답이 ASCII 형식이므로 2글자씩 해석하여 16진수로 변환
            response_hex = response.hex()
            print(f"응답 데이터 (16진수): {response_hex}")

            # 슬레이브 ID와 기능 코드 확인 (Modbus ASCII 응답 형식)
            slave_id = int(response_hex[0:2], 16)
            function_code = int(response_hex[2:4], 16)

            print(f"슬레이브 ID: {slave_id}, 기능 코드: {function_code}")

            # 응답 데이터 처리
            if function_code == 0x0F:  # 예시: 쓰기 명령에 대한 응답
                print("쓰기 명령에 대한 응답")
            else:
                print("알 수 없는 응답 형식입니다.")
        else:
            print("응답이 없습니다.")
    except Exception as e:
        print(f"응답 처리 중 오류 발생: {e}")

# 시리얼 포트 설정
ser = serial.Serial('COM4', baudrate=9600, timeout=1)

# 보내는 프레임 (ASCII 형식으로 전송)
frame = ":010F03F2000701648F"  # ASCII 형식 프레임

send_ascii_command(ser, frame)

# 시리얼 포트 닫기
ser.close()
