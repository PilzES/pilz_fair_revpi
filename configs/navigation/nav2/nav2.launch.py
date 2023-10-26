import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_yaml = os.path.join(
        get_package_share_directory('pilz_fair_revpi'), 'configs/navigation/nav2/controller.yaml')
    default_bt_xml_path = os.path.join(
        get_package_share_directory('pilz_fair_revpi'), 'configs/navigation/nav2/behavior_tree.xml')
    planner_yaml = os.path.join(
        get_package_share_directory('pilz_fair_revpi'), 'configs/navigation/nav2/planner_server.yaml')
    recovery_yaml = os.path.join(
        get_package_share_directory('pilz_fair_revpi'), 'configs/navigation/nav2/recovery.yaml')
    bt_navigator_yaml = os.path.join(
        get_package_share_directory('pilz_fair_revpi'), 'configs/navigation/nav2/bt_navigator.yaml')
    waypoint_follower = os.path.join(
        get_package_share_directory('pilz_fair_revpi'), 'configs/navigation/nav2/waypoint_follower.yaml')
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=remappings
        ),   
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml],
            remappings=remappings    
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[recovery_yaml],
            remappings=remappings    
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'default_bt_xml_filename': default_bt_xml_path}],
            remappings=remappings    
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[waypoint_follower ],
            remappings=remappings
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator',
                                        'waypoint_follower']}]
        )
    ])