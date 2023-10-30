import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_launch_file = os.path.join(
        get_package_share_directory('pilz_fair_revpi'), 'configs/navigation/nav2')
    
    amcl_launch_file = os.path.join(
        get_package_share_directory('pilz_fair_revpi'), 'configs/navigation/amcl')
    
    display_amv_launch_file = os.path.join(
        get_package_share_directory('pilz_fair_revpi'), 'launch')
    
    return LaunchDescription([
        Node(
            package="pilz_fair_revpi",
            executable="nav_waypoints_follower.py",
            name='nav_waypoints_follower',
            output="screen"
        ),

        Node(
            package='pilz_fair_revpi',
            executable='shared_memory',
            name='shared_memory',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([display_amv_launch_file, '/display_amv.launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file, '/nav2.launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([amcl_launch_file, '/amcl.launch.py'])
        )
        
    ])