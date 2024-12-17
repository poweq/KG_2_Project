import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    rviz_config_file = os.path.join(share_dir, 'config', 'ydlidar_cartographer.rviz')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                    default=os.path.join(share_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='ydlidar_cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([
        # 정적 변환: base_link -> laser_frame
#        Node(
 #           package='tf2_ros',
  #          executable='static_transform_publisher',
   #         output='screen',
    #        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser_frame']
     #   ),
        
        # 정적 변환: base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link']
        ),
        
        # 동적 변환: map -> odom, odom -> base_footprint
        Node(
            package='ydlidar_ros2_driver',
            executable='dynamic_tf_broadcaster.py',
            output='screen',
            name='dynamic_tf_broadcaster',
            parameters=[],
            prefix='python3'
        ),
        
        # Cartographer 실행
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename],
            remappings=[('odom', '/odom')]
        ),
        
        # Launch Argument: resolution
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'
        ),
        
        # Launch Argument: publish_period_sec
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'
        )
    ])
