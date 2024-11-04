import time
from pymycobot.mycobot import MyCobot

# MyCobot 포트 설정
mycobot = MyCobot("COM6", 115200)  # 포트와 baud rate는 설정에 맞게 조정

# 경로 설정: 이동할 좌표들
path_coords = [
    [242.8, -61.8, 282.9, 177.19, -0.19, -85.86],  # 첫 번째 위치
    [242.8, 61.8, 282.9, 177.19, -0.19, -85.86],  # 두 번째 위치
    [270.0, 61.8, 282.9, 177.19, -0.19, -85.86]  # 세 번째 위치
]

# 각 위치로 이동 함수
def move_to_position(coords, speed=20):
    x, y, z, rx, ry, rz = coords
    print(f"Moving to position: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    mycobot.send_coords([x, y, z, rx, ry, rz], speed)
    time.sleep(3)  # 이동 시간 대기 (필요시 조정 가능)

# 경로를 따라 이동 (세 번째 위치까지만 이동)
for i in range(3):
    move_to_position(path_coords[i])

# 세 번째 위치까지 도달 후 초기 조인트 위치로 복귀
mycobot.send_angles([0, 0, 0, 0, 0, 0], 20)  # 초기 조인트 각도로 복귀
print("초기 조인트 위치로 복귀했습니다!")

# import time
# from pymycobot.mycobot import MyCobot

# # MyCobot 포트 설정
# mycobot = MyCobot("COM6", 115200)  # 포트와 baud rate는 설정에 맞게 조정

# # 각 위치로 이동 함수
# def move_to_position(coords, speed=20):
#     x, y, z, rx, ry, rz = coords
#     print(f"Moving to position: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
#     mycobot.send_coords([x, y, z, rx, ry, rz], speed)
#     time.sleep(3)  # 이동 시간 대기 (필요시 조정 가능)

# # 첫 번째 위치로 이동
# move_to_position([200.0, 0.0, 50.0, 180.0, 0.0, 0.0])

# # 두 번째 위치로 이동
# move_to_position([200.0, 50.0, 50.0, 180.0, 0.0, 0.0])

# # 세 번째 위치로 이동
# move_to_position([200.0, -50.0, 50.0, 180.0, 0.0, 0.0])

# # 초기 조인트 위치로 복귀
# mycobot.send_angles([0, 0, 0, 0, 0, 0], 20)  # 초기 조인트 각도로 복귀
# print("초기 조인트 위치로 복귀했습니다!")
