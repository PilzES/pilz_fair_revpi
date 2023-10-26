
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
  
    robot_description_launch_file = os.path.join(
        get_package_share_directory('pilz_fair_revpi'),'configs/robot_description/robot_description.launch.py')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch_file])),

        Node(
            package="pilz_fair_revpi",
            executable="odom.py",
            name='odom',
            output="screen"
        ),
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"]
        ),
  	])