from pymycobot.mycobot import MyCobot
import time

mc = MyCobot('COM6',115200)
# /dev/ttyACM0
#기본 모드 0
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()

mc.set_gripper_state(1,20,1) #닫기 
time.sleep(3)
mc.set_gripper_state(0,20,1) #열기
time.sleep(3)
# mc.set_gripper_state(0,20,1) #열기

#위치 제어 모드1
# mc.set_gripper_mode(1)
# mc.set_gripper_value(100, 20)  # 그리퍼를 완전히 닫음 (예: 100이 완전 닫힘 위치)
# time.sleep(2)
# mc.set_gripper_value(0, 20)  # 그리퍼를 완전히 염 (예: 0이 완전 열림 위치)


# 속도 제어 모드2
# mc.set_gripper_mode(2)  # 속도 제어 모드 설정
# mc.set_gripper_state(1, 10, 1)  # 그리퍼를 느린 속도로 닫기
# time.sleep(2)
# mc.set_gripper_state(0, 50, 1)  # 그리퍼를 빠른 속도로 열기


