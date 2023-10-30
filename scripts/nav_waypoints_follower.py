#! /usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from copy import deepcopy

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    route = [[5.0, 0.0, 0.68, 0.73],
            [5.0, 5.0, -1.0, 0.0],
            [-5.0, 5.0, -0.72, 0.7],
            [-5.0, -5.0, 0.0, 1.0],
            [5.0, -5.0, 0.92, 0.38],
            [-5.0, 5.0, 0.0, 1.0],
            [5.0, 5.0, -0.72, 0.69],
            [5.0, -5.0, -1.0, 0.0],
            [-5.0, -5.0, 0.46, 0.89],
            [0.0, 0.0, 0.0, 1.0]]
    
    while rclpy.ok():
        goal_poses = []
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        for point in route:
            goal_pose.pose.position.x = point[0]
            goal_pose.pose.position.y = point[1]
            goal_pose.pose.orientation.z = point[2]
            goal_pose.pose.orientation.w = point[3]
            goal_poses.append(deepcopy(goal_pose))
        navigator.followWaypoints(goal_poses)

        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()