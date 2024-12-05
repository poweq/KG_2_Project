from pymodbus.client.sync import ModbusSerialClient as ModbusClient

# Modbus RTU 통신 설정
client = ModbusClient(
    method='rtu',
    port='COM4',  # RS-485가 연결된 PC의 COM 포트
    baudrate=9600,  # PLC 통신 속도와 동일 (예: 9600 bps)
    parity='N',  # Parity 설정 (None: 'N', Even: 'E', Odd: 'O')
    stopbits=1,  # Stop bit 설정 (1 또는 2)
    bytesize=8,  # 데이터 비트 (일반적으로 8)
    timeout=1  # 타임아웃 시간 (초 단위)
)

# PLC 연결 시도
if client.connect():
    print("Modbus RTU 연결 성공")

    # %M10 (Modbus 주소 10번) 코일 상태 읽기
    response = client.read_holding_registers(address=0, count=2, unit=1)  # 슬레이브 주소 = 1
    if response.isError():
        print(f"코일 값 읽기 실패: {response}")
    else:
        # %M10 상태 출력 (ON: True, OFF: False)
        print(f"%M10 상태: {response.registers[0]}")  # response.bits는 Boolean 리스트
    
    # 연결 종료
    client.close()
else:
    print("Modbus RTU 연결 실패")
