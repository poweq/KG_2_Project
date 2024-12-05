import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 이름 설정 (패키지 이름 확인)
    package_name = 'aseembly'  # 실제 패키지 이름으로 수정

    # URDF 파일 경로 설정
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'aseembly.urdf')  # 실제 URDF 파일명으로 수정

    # URDF 파일 내용 읽기
    with open(urdf_file, 'r') as urdf:
        robot_description_content = urdf.read()

    # 'robot_description' 파라미터 설정
    robot_description = {'robot_description': robot_description_content}

    # robot_state_publisher 노드 정의
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # LaunchDescription 반환
    return LaunchDescription([
        robot_state_publisher_node
    ])
