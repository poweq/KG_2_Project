import serial
import time
import logging
from pymycobot.log import setup_logging
from pymycobot.common import DataProcessor
from pymycobot.error import calibration_parameters
from enum import Enum

class ProtocolCode(Enum):
    HEADER = 0xFE  # 헤더 코드
    RESTORE = [0x01, 0x00]
    SET_LED = [0x01, 0x02]
    GET_FIRMWARE_VERSION = [0x01, 0x03]
    GET_MOTORS_CURRENT = [0x01, 0x04]
    GET_BATTERY_INFO = [0x01, 0x05]
    SET_GYRO_STATE = [0x01, 0x07]
    GET_GYRO_STATE = [0x01, 0x08]
    GET_MODIFIED_VERSION = [0x01, 0x09]

class MyAgv(DataProcessor):
    def __init__(self, port="/dev/ttyAMA2", baudrate=115200, timeout=0.1, debug=False):
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)
        self._serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout
        )
        
        # 포트가 이미 열려 있는지 확인하고 처리
        if not self._serial_port.is_open:
            self._serial_port.open()

        self._serial_port.rts = False
        self.__movement = False

    def go_ahead(self, speed=50, timeout=5):
        """
        AGV를 직진시키는 명령을 전송합니다.
        
        Args:
            speed (int): 1 ~ 127 사이의 속도 값
            timeout (float): 동작 지속 시간 (초 단위)
        """
        if not (1 <= speed <= 127):
            raise ValueError("속도는 1과 127 사이여야 합니다.")

        command = [128 + speed, 128, 128]  # 직진 명령 구성
        self.__basic_move_control(command, timeout=timeout)

    def __basic_move_control(self, command, timeout=5):
        """
        일정 시간 동안 0.05초 간격으로 명령을 반복 전송합니다.
        
        Args:
            command: 전송할 명령 리스트
            timeout (float): 동작 지속 시간 (초 단위)
        """
        t = time.time()
        self.__movement = True

        while time.time() - t < timeout and self.__movement:
            self._mesg(command)  # 명령 전송
            time.sleep(0.05)  # 50ms 주기

        self.stop()

    def _mesg(self, genre, *args, **kwargs):
        """
        명령을 프로토콜에 맞게 구성하고 전송합니다.
        
        Args:
            genre: 명령 종류 (list 또는 단일 명령)
            *args: 추가 인수
            **kwargs: 응답 여부 (has_reply)
        """
        has_reply = kwargs.get("has_reply", None)
        real_command = self._process_data_command(genre, self.__class__.__name__, args)

        command = [ProtocolCode.HEADER.value, ProtocolCode.HEADER.value]
        if isinstance(genre, list):
            command.extend(genre)
        else:
            command.append(genre)
        command.append(real_command)
        command = self._flatten(command)

        # 체크섬 추가
        command.append(sum(command[2:]) & 0xFF)

        self._write(command)

        if has_reply:
            data = self._read(command)
            return data if data else None

    def stop(self):
        """AGV를 멈추는 명령을 전송합니다."""
        stop_command = [128, 128, 128]
        self._mesg(stop_command)
        self.__movement = False

    def _write(self, command):
        """시리얼 포트로 명령을 전송합니다."""
        self._serial_port.reset_input_buffer()
        self.log.debug(f"_write: {[hex(i) for i in command]}")
        self._serial_port.write(command)
        self._serial_port.flush()

    def _read(self, command) -> bytes:
        """시리얼 포트에서 응답을 읽어옵니다."""
        data = self._serial_port.read(size=7)  # 응답 길이 7바이트로 가정
        self.log.debug(f"_read: {[hex(i) for i in data]}")
        return data

    def close(self):
        """시리얼 포트를 닫습니다."""
        if self._serial_port.is_open:
            self._serial_port.close()

# 사용 예시
if __name__ == "__main__":
    agv = MyAgv()
    try:
        # AGV를 50 속도로 3초 동안 직진
        agv.go_ahead(speed=20, timeout=0.3)
    finally:
        agv.stop()
        agv.close()
