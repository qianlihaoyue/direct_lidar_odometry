import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('direct_lidar_odometry')
    # siasun 
    config_file = os.path.join(package_dir, 'config','params.yaml')
    rviz_file = os.path.join(package_dir, 'config', 'rviz.rviz')

    odom_node = Node(
        package='direct_lidar_odometry',
        executable='dlo_odom_node',
        parameters=[config_file],
        output='screen'
    )
    map_node = Node(
        package='direct_lidar_odometry',
        executable='dlo_map_node',
        parameters=[config_file],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
    )

    ld = LaunchDescription()
    ld.add_action(odom_node)
    ld.add_action(map_node)
    ld.add_action(rviz_node)

    return ld
