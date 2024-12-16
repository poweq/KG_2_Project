cmake_minimum_required(VERSION 3.5)
project(myagv_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
