from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(port='COM4', baudrate=9600, timeout=1)

if not client.connect():
    print("클라이언트 연결 오류")
else:
    print("클라이언트에 성공적으로 연결되었습니다.")

coil_address = 4  # PLC P0504번 주소
coil_value = True # 코일 값을 True로 설정 ON. False는 OFF.

# 코일 값을 변경하기 위한 쓰기 요청
response = client.write_coil(coil_address, coil_value)

if response.isError():
    print("코일 값 변경 오류:", response)
else:
    print("코일 값이 성공적으로 변경되었습니다.")

client.close()
