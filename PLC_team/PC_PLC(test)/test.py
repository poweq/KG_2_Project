import serial

# 1. 시리얼 포트 설정
ser = serial.Serial(
    port='COM4',          # 사용하는 COM 포트 이름
    baudrate=9600,        # 통신 속도(PLC의 설정과 일치해야 함)
    parity=serial.PARITY_NONE,  # 패리티 설정(없음)
    stopbits=serial.STOPBITS_ONE,  # 스톱 비트 설정
    bytesize=serial.EIGHTBITS,  # 데이터 비트 설정
    timeout=2             # 읽기 타임아웃(초 단위)
)

# 2. 명령 프레임 전송 (Modbus RTU 예제 프레임)
tx_frame = b'\x01\x01\x00\x00\x00\x08\xAD'  # 명령 프레임 (필요에 따라 수정)

# 3. 데이터 전송
ser.write(tx_frame)
print("보낸 프레임:", tx_frame)

# 응답 받기
try:
    print("응답 대기 중...")
    response = ser.read(8)  # PLC에서 오는 응답 데이터 길이 (8바이트로 가정)
    print("수신한 응답:", response)

    # 응답을 16진수 형식으로 출력
    print("16진수 형식:", response.hex())
except Exception as e:
    print("응답 읽기 오류:", e)
finally:
    ser.close()
    print("시리얼 포트를 닫았습니다.")
