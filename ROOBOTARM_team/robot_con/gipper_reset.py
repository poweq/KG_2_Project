from pymycobot.mycobot import MyCobot
import time

mc = MyCobot('COM6',115200)
# /dev/ttyACM0
#기본 모드 0 (설정값으로 제어- > 1: 닫힘, 0: 열림)
# mc.set_gripper_mode(0)
# mc.init_gripper()
# mc.set_gripper_calibration()

# mc.set_gripper_state(1,20,1) #닫기 
# time.sleep(3)
# mc.set_gripper_state(0,20,1) #열기
# time.sleep(3)

# mc.set_gripper_state(0,20,1) #열기

#위치 제어 모드1 (범위로 제어- > 0 ~ 100, 0: 완전히 닫힘, 100: 완전히 열림)
mc.set_gripper_mode(0)
mc.init_gripper()
mc.init_eletric_gripper()
time.sleep(1)
mc.set_gripper_value(15,20,1) # 그리퍼를 완전히 닫음 (0 ~ 100, 0 이 완전히 닫힘)
time.sleep(3)
mc.set_gripper_value(100,20,1)# 그리퍼를 완전히 열림 (0 ~ 100, 100 이 완전히 열림)
time.sleep(3)

# 속도 제어 모드2
# mc.set_gripper_mode(0)  # 속도 제어 모드 설정
# mc.init_gripper()
# mc.set_gripper_calibration()
# mc.set_gripper_state(1, 10, 1)  # 그리퍼를 느린 속도로 닫기
# time.sleep(2)
# mc.set_gripper_state(0, 50, 1)  # 그리퍼를 빠른 속도로 열기