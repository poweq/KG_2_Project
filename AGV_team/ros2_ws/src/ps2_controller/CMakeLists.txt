cmake_minimum_required(VERSION 3.5)
project(ps2_controller)

# 필요한 패키지 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# include 디렉토리 추가
include_directories(
  include
)

# 실행 파일 생성 (두 개의 소스 파일 포함)
add_executable(ps2_controller_node
  src/ps2_controller_node.cpp
  src/ps2_controller.cpp
)

# 헤더 파일 경로 설정
target_include_directories(ps2_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 실행 파일과 의존성 연결
ament_target_dependencies(ps2_controller_node
  rclcpp
  geometry_msgs
)

# 설치 규칙
install(TARGETS
  ps2_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
