#! /usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from copy import deepcopy

class MyNavigator:
    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()

    def set_initial_pose(self, x, y, orientation_z, orientation_w):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.orientation.z = orientation_z
        initial_pose.pose.orientation.w = orientation_w
        self.navigator.setInitialPose(initial_pose)

    def follow_route(self, route):
        goal_poses = []
        for point in route:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = point[0]
            goal_pose.pose.position.y = point[1]
            goal_pose.pose.orientation.z = point[2]
            goal_pose.pose.orientation.w = point[3]
            goal_poses.append(deepcopy(goal_pose))

        self.navigator.followWaypoints(goal_poses)

        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                      str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))

        result = self.navigator.getResult()
        return result

    def shutdown(self):
        self.navigator.lifecycleShutdown()

def main():
    navigator = MyNavigator()
    navigator.set_initial_pose(0.0, 0.0, 0.0, 0.0)

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
        result = navigator.follow_route(route)

    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.shutdown()

if __name__ == '__main__':
    main()
