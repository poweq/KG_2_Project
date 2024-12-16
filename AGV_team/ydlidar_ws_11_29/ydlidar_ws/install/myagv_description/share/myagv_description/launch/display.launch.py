import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 'model' 인자 선언
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=''
    )

    # aseembly 패키지의 공유 디렉토리 경로 가져오기
    aseembly_share_dir = get_package_share_directory('aseembly')

    # URDF 파일의 경로 설정
    urdf_file = os.path.join(aseembly_share_dir, 'urdf', 'aseembly.urdf')

    # URDF 파일 내용 읽기
    with open(urdf_file, 'r') as urdf:
        robot_description_content = urdf.read()

    # 'robot_description' 파라미터 설정
    robot_description = {'robot_description': robot_description_content}

    # RViz 설정 파일 경로 설정
    rviz_config_file = os.path.join(aseembly_share_dir, 'urdf.rviz')

    # 노드 정의
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file]
    )

    # LaunchDescription에 모든 요소 추가
    return LaunchDescription([
        model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
