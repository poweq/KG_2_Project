import serial
import time

# ESP32와의 직렬 포트 설정 (포트 이름은 실제 사용 환경에 맞게 수정)
ser = serial.Serial('COM9', 115200)  # COMx는 실제 ESP32의 포트 번호로 바꿔야 함

def set_servo_angle(angle):
    if angle == 0:
        ser.write(b't')  # t는 0도로 설정
    elif angle == 90:
        ser.write(b'f')  # f는 90도로 설정

def main():
    while True:
        user_input = input("서보모터 각도를 설정하세요 (t: 0도, f: 90도, q: 종료): ")
        
        if user_input == 't':
            set_servo_angle(0)
        elif user_input == 'f':
            set_servo_angle(90)
        elif user_input == 'q':
            print("프로그램을 종료합니다.")
            break
        else:
            print("잘못된 입력입니다. 다시 시도하세요.")

        time.sleep(0.5)

if __name__ == "__main__":
    main()
