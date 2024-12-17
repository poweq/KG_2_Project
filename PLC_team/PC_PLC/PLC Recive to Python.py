import serial

# 시리얼 포트와 통신 속도 설정
ser = serial.Serial(
    port='COM4',       # PC에서 사용 중인 포트를 설정 (예: COM4)
    baudrate=9600,     # PLC와 동일한 통신 속도로 설정
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# PLC로부터 HEX 값을 수신
response = ser.read(14)  # 14바이트 수신 (0A 31 30 30 30 31 30 30 31 31 30 30 32 0A)

# 수신된 값을 HEX 형식으로 변환하여 출력
hex_output = response.hex().upper()
formatted_output = ' '.join(hex_output[i:i+2] for i in range(0, len(hex_output), 2))
print("수신된 HEX 값:", formatted_output)

# 수신한 데이터를 문자열로 변환
string_output = response.decode('ascii', errors='ignore')  # ASCII로 변환, 변환 불가능한 문자는 무시
print("변환된 문자열:", string_output)

# 문자열에서 숫자만 추출하여 출력
numbers = ''.join(filter(str.isdigit, string_output))
print("추출된 숫자:", numbers)

ser.close()  # 시리얼 포트 닫기
