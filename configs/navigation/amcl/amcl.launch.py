import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    amcl_yaml = os.path.join(
        get_package_share_directory('pilz_fair_revpi'), 'configs/navigation/amcl/amcl_config.yaml')
    map_file = os.path.join(
        get_package_share_directory('pilz_fair_revpi'), 'maps/neo_track1.yaml')
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_file}],
            remappings=remappings
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_yaml],
            remappings=remappings
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl']}]
        )
    ])