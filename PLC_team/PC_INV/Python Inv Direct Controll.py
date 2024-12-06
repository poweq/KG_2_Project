from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException
import time


class InverterController:
    def __init__(self, port, baudrate):
        self.client = ModbusSerialClient(
            method="rtu",
            port=port,
            baudrate=baudrate,
            parity="N",
            stopbits=1,
            timeout=3,  # 타임아웃을 3초로 증가
        )
        if self.client.connect():
            print("Modbus connection successful!")
        else:
            print("Modbus connection failed!")
            raise ConnectionError("Failed to connect to the Modbus device.")

    def set_frequency(self, frequency_hz):
        """주파수 설정 (Hz)"""
        frequency_value = int(frequency_hz * 100)  # 0.01Hz 단위 변환
        try:
            response = self.client.write_register(4, frequency_value, unit=1)  # 0x0005: 주파수 설정
            print(f"주파수 설정 응답: {response}")
            return response
        except Exception as e:
            print(f"주파수 설정 오류: {e}")
            return None

    def set_direction(self, direction):
        """
        방향 및 정지 설정
        - 1: 정방향 운전
        - 2: 역방향 운전
        - 0: 정지
        """
        # 매뉴얼에 따른 명령값 정의
        if direction == 1:
            command_value = 0x13C2  # 정방향 운전
        elif direction == 2:
            command_value = 0x13C4  # 역방향 운전
        elif direction == 0:
            command_value = 0x13C1  # 정지
        else:
            raise ValueError("Direction must be 0 (Stop), 1 (Forward), or 2 (Reverse).")

        try:
            response = self.client.write_register(5, command_value, unit=1)  # 0x0006: 방향 설정
            print(f"방향 설정 응답: {response}")
            return response
        except Exception as e:
            print(f"방향 설정 오류: {e}")
            return None

    def read_register(self, register_address):
        """레지스터 값 읽기"""
        try:
            response = self.client.read_holding_registers(register_address, 1, unit=1)
            if isinstance(response, ModbusIOException):
                print(f"레지스터 {register_address} 읽기 오류: {response}")
                return None
            print(f"레지스터 {register_address} 읽기 응답: {response.registers}")
            return response.registers[0]
        except Exception as e:
            print(f"레지스터 {register_address} 읽기 오류: {e}")
            return None

    def close(self):
        """Modbus 연결 종료"""
        self.client.close()


# 메인 실행
if __name__ == "__main__":
    port = "COM4"  # Windows에서 COM 포트
    baudrate = 9600

    try:
        controller = InverterController(port, baudrate)

        # 1. 속도 설정 (예: 40Hz)
        if controller.set_frequency(35) is None:
            print("주파수 설정 실패.")

        while True:
            try:
                # 사용자 입력
                user_input = input("방향을 입력하세요 (0: 정지, 1: 정방향, 2: 역방향, q: 종료): ").strip()
                
                if user_input.lower() == 'q':  # 'q' 입력 시 종료
                    print("프로그램을 종료합니다.")
                    break

                direction_input = int(user_input)  # 숫자로 변환
                if direction_input not in [0, 1, 2]:
                    raise ValueError("유효하지 않은 방향 값입니다. 0, 1, 2 중 하나를 입력하세요.")

                # 방향 설정
                if controller.set_direction(direction_input) is None:
                    print("방향 설정 실패.")
                else:
                    print("방향 설정 성공.")
            except ValueError as ve:
                print(f"오류: {ve}")

        # 2. 현재 설정된 주파수 및 방향 읽기
        current_frequency = controller.read_register(4)  # 주파수 레지스터
        if current_frequency is not None:
            print(f"현재 설정된 주파수: {current_frequency / 100} Hz")
        else:
            print("주파수 읽기 실패.")

        current_direction = controller.read_register(5)  # 방향 레지스터
        if current_direction is not None:
            print(f"현재 설정된 방향: {current_direction}")
        else:
            print("방향 읽기 실패.")

    except Exception as e:
        print(f"오류 발생: {e}")

    finally:
        controller.close()
