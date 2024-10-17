from pymycobot.mycobot import MyCobot
import time

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)

# 동작 선택 함수
def perform_action(action):
    if action == 'a':
        print("A 동작 실행")
        # A동작 코드
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)
        time.sleep(1)

        mc.send_angles([72.07, 41.66, 69.02, -28.53, -83.93, -22.76], 20)
        time.sleep(3)

        mc.set_gripper_state(1,20,1)
        time.sleep(1)

        mc.send_angles([72.07, 20.66, 69.02, -28.53, -83.93, -22.76], 20)
        time.sleep(1)

        mc.send_angles([20.19, 30.24, 7.84, -17.39, -83.06, 24], 20)
        time.sleep(3)

        mc.send_angles([16.19, 75.24, 7.84, -17.39, -83.06, 24], 20)
        time.sleep(3)

        mc.set_gripper_state(0,20,1)
        time.sleep(3)

        mc.send_angles([0,0,0,0,0,0],20)
        time.sleep(3)

    elif action == 'b':
        print("B 동작 실행")
        # B동작 코드
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)
        time.sleep(1)

        mc.send_angles([72.07, 41.66, 69.02, -28.53, -83.93, -22.76], 20)
        time.sleep(3)

        mc.set_gripper_state(1,20,1)
        time.sleep(1)

        mc.send_angles([72.07, 20.66, 69.02, -28.53, -83.93, -22.76], 20)
        time.sleep(1)

        mc.send_angles([3, 22, 37.84, -21.39, -90, 6.08], 20)
        time.sleep(3)

        mc.send_angles([2, 58, 38.84, -21.39, -90, 3], 20)
        time.sleep(3)

        mc.set_gripper_state(0,20,1)
        time.sleep(3)

        mc.send_angles([0,0,0,0,0,0],20)
        time.sleep(3)

    elif action == 'c':
        print("C 동작 실행")
        # C동작 코드
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)
        time.sleep(1)

        mc.send_angles([72.07, 41.66, 69.02, -28.53, -83.93, -22.76], 20)
        time.sleep(3)

        mc.set_gripper_state(1,20,1)
        time.sleep(1)

        mc.send_angles([72.07, 20.66, 69.02, -28.53, -83.93, -22.76], 20)
        time.sleep(1)

        mc.send_angles([-19, 20, 1, 12, -90, -22], 20)
        time.sleep(3)

        mc.send_angles([-19, 76, 1, 12, -90, -22], 20)
        time.sleep(3)

        mc.set_gripper_state(0,20,1)
        time.sleep(3)

        mc.send_angles([0,0,0,0,0,0],20)
        time.sleep(3)

    else:
        print("잘못된 입력입니다. 'a', 'b', 'c' 중 하나를 입력하세요.")

# 메인 루프
while True:
    action = input("실행할 동작을 선택하세요 (a, b, c) \n종료를 원하시면 'q'를 입력하세요.\n 실행 시킬 동작 :").lower()
    if action in ['a', 'b', 'c']:
        perform_action(action)
    else:
        if action == 'q':
            print("프로그램이 종료되었습니다.")
            break
