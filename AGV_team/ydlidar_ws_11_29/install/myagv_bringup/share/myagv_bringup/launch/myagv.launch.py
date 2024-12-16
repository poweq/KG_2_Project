import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyAMA2')
    
    # myAGV param 설정
    myagv_param_dir =LaunchConfiguration(
        'myagv_param_dir',
        default=os.path.join(
        get_package_share_directory('myagv_navigation2'),
        'param',
        'myAGV.yaml'))

    # YDLidar pkg 경로 설정
    ydlidar_launch_file_dir = LaunchConfiguration(
        'ydlidar_launch_file_dir',
        default=os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch')
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'
        ),

        DeclareLaunchArgument(
            'myagv_param_dir',
            default_value=myagv_param_dir,
            description='Full path to turtlebot3 parameter file to load'
        ),
        
        # Include the launch file that starts the robot state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/myagv_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        # Include the launch file that starts the ydlidar launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ydlidar_launch_file_dir, '/ydlidar_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Include the launch file that starts the teleop node
        # myAGV Node 실행
        Node(
            package='jdamr200_node',
            executable='jdamr200_node',
            parameters=[myagv_param_dir],
            arguments=['-i', usb_port],
            output='screen',
        )
    ])
