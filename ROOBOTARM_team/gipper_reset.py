from pymycobot.mycobot import MyCobot
import time

mc = MyCobot('COM6',115200)
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()

mc.set_gripper_state(0,20,1) #열기 
time.sleep(3)
mc.set_gripper_state(1,20,1) #닫기
time.sleep(3)
mc.set_gripper_state(0,20,1) #열기

