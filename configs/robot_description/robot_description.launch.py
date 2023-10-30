
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  urdf_path = PathJoinSubstitution([FindPackageShare("pilz_fair_revpi"), "urdf/amv.urdf.xacro"])
      
  return LaunchDescription([
    DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),
        
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[
              {
                  'robot_description': Command(['xacro ', LaunchConfiguration("urdf")])
              }
          ]
      	)
    ])