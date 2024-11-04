# 변환 계수 (계산된 값 사용)
coeff_x = -16.67  # 도/픽셀
coeff_y = -0.806  # 도/픽셀

# 목표 중심 좌표 (이미지 중앙)
TARGET_X = 320  # 이미지 너비의 절반
TARGET_Y = 240  # 이미지 높이의 절반

# 현재 객체 중심 좌표 (예시)
x_center = ...  # 객체 감지 결과로부터 얻음
y_center = ...  # 객체 감지 결과로부터 얻음

# 픽셀 이동량 계산
delta_x_pixel = TARGET_X - x_center
delta_y_pixel = TARGET_Y - y_center

# 각도 변화량 계산
delta_theta1 = coeff_x * delta_x_pixel
delta_theta2 = coeff_y * delta_y_pixel

# 현재 로봇 암 각도 가져오기
current_angles = mc.get_angles()
theta1_current = current_angles[0]
theta2_current = current_angles[1]

# 목표 각도 계산
theta1_target = theta1_current + delta_theta1
theta2_target = theta2_current + delta_theta2

# 로봇 암 제어 명령 전송
mc.send_angles([theta1_target, theta2_target, ... 나머지 각도 ...], 20)
