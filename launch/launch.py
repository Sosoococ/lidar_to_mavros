# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='lidar_to_mavros',
#             executable='lidar_odom_to_mavros',
#             name='lidar_odom_to_mavros',
#             parameters=['config/config.yaml'],
#             output='screen'
#         )
#     ])
    
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    
    config = os.path.join(
            get_package_share_directory('lidar_to_mavros'),
            'config',
            'config.yaml'
            ) 
    node = Node(
        package='lidar_to_mavros',
        executable='lidar_odom_to_mavros',
        output='screen',
        parameters=[config]
    )
    
    ld.add_action(node)
    
    return ld
    