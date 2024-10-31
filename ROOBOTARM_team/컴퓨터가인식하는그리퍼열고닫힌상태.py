from pymycobot.mycobot import MyCobot
import time

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# 그리퍼 상태를 확인하는 함수
def is_gripper_closed():
    gripper_position = mc.get_gripper_value()  # 그리퍼 위치 가져오기
    if gripper_position <= 0:  # 위치가 10 이하일 때 닫혀있다고 가정 (예제 기준)
        return True
    else:
        return False

# 그리퍼 상태 확인
if is_gripper_closed():
    print("그리퍼가 닫혀 있습니다.")
else:
    print("그리퍼가 열려 있습니다.")