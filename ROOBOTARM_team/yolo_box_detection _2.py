from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO
import threading

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# YOLO 모델 로드
model = YOLO('C:\\Users\\shims\\Desktop\\github\\yolo_dectect_traing\\runs\\detect\\train\\weights\\best.pt')

# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)

