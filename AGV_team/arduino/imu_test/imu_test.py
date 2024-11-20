import serial
import time

# 라즈베리 파이의 ttyAMA1 포트 설정 (아두이노와 연결된 포트)
ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
time.sleep(2)  # 시리얼 포트 초기화 대기

def parse_imu_data(line):
    try:
        # 문자열을 ','로 분리하여 각 값을 리스트로 저장
        data = line.split(',')
        if len(data) == 7 and data[0] == "242":  # 0xf2의 10진수 표현인 242 확인
            gyro_x = float(data[1])
            gyro_y = float(data[2])
            gyro_z = float(data[3])
            accel_x = float(data[4])
            accel_y = float(data[5])
            accel_z = float(data[6])

            # 각속도 및 가속도 값 출력
            print(f"Gyro X: {gyro_x}, Gyro Y: {gyro_y}, Gyro Z: {gyro_z}")
            print(f"Accel X: {accel_x}, Accel Y: {accel_y}, Accel Z: {accel_z}")
        else:
            print("잘못된 데이터 형식이거나 0xf2가 아님. 무시합니다.")
    except ValueError:
        print("데이터 파싱 오류 발생. 무시합니다.")

try:
    while True:
        if ser.in_waiting > 0:
            try:
                # 아두이노로부터 한 줄 읽기
                line = ser.readline().decode('utf-8').strip()
                print(f"수신된 데이터: {line}")  # 수신된 원본 데이터 출력
                parse_imu_data(line)
            except UnicodeDecodeError:
                print("디코딩 오류 발생. 잘못된 데이터 무시.")
except KeyboardInterrupt:
    print("프로그램 종료")
finally:
    ser.close()
