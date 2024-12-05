from pymodbus.client.sync import ModbusSerialClient
import logging

# 로깅 설정
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# Modbus 클라이언트 설정
client = ModbusSerialClient(
    method='rtu',
    port='COM4',       # PLC가 연결된 COM 포트
    baudrate=9600,     # PLC 통신 속도
    parity='N',        # 패리티 (None)
    stopbits=1,        # 정지 비트
    bytesize=8,        # 데이터 비트
    timeout=3          # 응답 대기 시간
)

# Modbus 연결
if not client.connect():
    print("클라이언트 연결 오류. PLC와 연결 확인 후 다시 시도하세요.")
    exit()

print("클라이언트에 성공적으로 연결되었습니다.")

try:
    # Coil 쓰기 테스트
    coil_address = 4  # XG5000에서 확인한 Coil 주소 (예: M100 -> Coil 4)
    coil_value = True  # True: ON, False: OFF
    
    response = client.write_coil(coil_address, coil_value, unit=1)  # unit=1은 Slave ID
    
    if response.isError():
        print(f"Coil 쓰기 오류: {response}")
    else:
        print(f"Coil {coil_address} 값이 {'ON' if coil_value else 'OFF'}로 설정되었습니다.")

except Exception as e:
    print(f"통신 중 예외 발생: {e}")

finally:
    # 클라이언트 연결 종료
    client.close()
    print("클라이언트 연결이 종료되었습니다.")
