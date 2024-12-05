from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np

# MyCobot 연결 설정
mc = MyCobot('/dev/ttyACM0', 115200)

# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)

# 작업 수행 함수
def perform_action():
    # 초기 위치로 이동 후 색상 탐지
    mc.send_angles([69, 70, 14, -2, -90, 0], 20)
    time.sleep(3)

    # 그리퍼 닫기 (블록 잡기)
    mc.set_gripper_state(1, 20, 1)
    time.sleep(3)

    # 초기 위치로 복귀
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    time.sleep(3)

    # mc.set_gripper_state(0, 20, 1)
    # time.sleep(3)

perform_action()



