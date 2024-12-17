import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_arg = DeclareLaunchArgument('model', default_value='')

    myagv_description_share_dir = get_package_share_directory('myagv_description')
    urdf_file = os.path.join(myagv_description_share_dir, 'urdf', 'myagv_description.urdf')

    with open(urdf_file, 'r') as urdf:
        robot_description_content = urdf.read()

    robot_description = {'robot_description': robot_description_content}
    rviz_config_file = os.path.join(myagv_description_share_dir, 'urdf.rviz')

    return LaunchDescription([
        model_arg,
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_file]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
            name='static_transform_map_to_odom'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_footprint'],
            name='static_transform_odom_to_base_footprint'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link'],
            name='static_transform_base_footprint_to_base_link'
        ),
    ])
