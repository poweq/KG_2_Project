import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'myagv_description'
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'myagv_description.urdf'
    )

    with open(urdf_file, 'r') as urdf:
        robot_description_content = urdf.read()

    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
    ])
