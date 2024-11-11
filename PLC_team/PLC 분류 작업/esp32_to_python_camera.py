import cv2  # opencv (설치필요)
import tkinter as tk
from tkinter import Label, Checkbutton, IntVar
from PIL import Image, ImageTk, ImageDraw, ImageFont  # Pillow모듈(설치필요)
import datetime
import time
import numpy as np  # numpy 설치필요

# 전역 FPS 값 설정 (초기값: 25)
FPS = 60

# ESP32-CAM MJPEG 스트림 URL
mjpeg_url = 'http://192.168.0.105:81/stream'  # 여기에 ESP32-CAM IP 주소와 포트를 입력하세요.

# tkinter 윈도우를 생성합니다.
root = tk.Tk()
root.title("ESP32-CAM Stream")

# Label 위젯을 생성하여 비디오 프레임을 표시합니다.
label = Label(root)
label.pack()

# 체크박스를 생성합니다.
show_time_var = IntVar()
# 체크를 했는지 안했는지를 show_time_var가 True인지 False인지로 알 수 있다!
show_time_checkbox = Checkbutton(root, text="현재시간추가", variable=show_time_var)
show_time_checkbox.pack()

# OpenCV를 사용하여 MJPEG 스트림을 가져옵니다.
cap = cv2.VideoCapture(mjpeg_url)
cap.set(cv2.CAP_PROP_FPS, FPS)  # FPS 설정
cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)  # 버퍼 크기 설정
cap.set(cv2.CAP_PROP_FFMPEG, 1)  # FFmpeg 사용 설정
cap.set(cv2.CAP_PROP_FFMPEG_TIMEOUT, 5000)  # 타임아웃 설정

# 원본 해상도를 가져옵니다.
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

def reconnect_stream():
    """스트림 연결이 끊겼을 때 재연결 시도"""
    global cap
    print("스트림을 재연결 중...")
    cap.release()
    time.sleep(2)  # 잠시 대기 후 재연결 시도
    cap = cv2.VideoCapture(mjpeg_url)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
    cap.set(cv2.CAP_PROP_FFMPEG_TIMEOUT, 5000)  # 타임아웃 설정

def update_frame():
    ret, frame = cap.read()  # ESP32가 보낸 영상을 읽는다!
    
    if not ret:
        print("프레임을 캡처하지 못했습니다. 재연결 시도 중...")
        reconnect_stream()
        return

    # OpenCV에서 읽은 이미지를 PIL 형식으로 변환합니다.
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(frame_rgb)
    
    # 체크박스 상태에 따라 현재 시간을 오버레이합니다.
    if show_time_var.get():
        draw = ImageDraw.Draw(img)
        # 폰트 크기 설정 (3배 크게)
        font_size = 30  # 기본 폰트 크기보다 크게 설정
        try:
            font = ImageFont.truetype("arial.ttf", font_size)
        except IOError:
            font = ImageFont.load_default()  # 폰트 파일을 찾을 수 없는 경우 기본 폰트 사용
        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        text_bbox = draw.textbbox((0, 0), now, font=font)
        text_width = text_bbox[2] - text_bbox[0]
        text_height = text_bbox[3] - text_bbox[1]
        text_x = frame_width - text_width - 10
        text_y = frame_height - text_height - 15  # 텍스트를 5픽셀 위로 이동
        draw.text((text_x, text_y), now, font=font, fill=(255, 255, 255))
    
    imgtk = ImageTk.PhotoImage(image=img)

    # Label에 이미지를 업데이트합니다.
    label.imgtk = imgtk
    label.configure(image=imgtk)
    
    # GUI 업데이트를 위한 루프 재귀 호출
    root.after(int(1000 / FPS), update_frame)  # FPS에 맞춰 업데이트 주기 설정

# 프레임 업데이트 루프를 시작합니다.
update_frame()

# tkinter 이벤트 루프를 시작합니다.
root.mainloop()

# 카메라 릴리스
cap.release()
