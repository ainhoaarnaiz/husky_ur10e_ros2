#! /usr/bin/env python3

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

start = {
    "point_a":[-3.829, -7.604],
    "point_b":[-3.791, -3.287],
    "point_c":[-3.791, 1.254],
    "point_d":[-3.24, 5.861],
}


end = {
    "point_x":[-0.205, 7.403],
    "point_y":[-0.073, -8.497],
    "point_w":[6.217, 2.153],
    "point_z":[-6.349, 9.147],
}

def main():

    start_location = 'point_c'
    end_location = 'point_y'

    rclpy.init()
    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.45
    initial_pose.pose.position.y = 2.15
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    item_pose = PoseStamped()
    item_pose.header.frame_id = 'map'
    item_pose.header.stamp = navigator.get_clock().now().to_msg()
    item_pose.pose.position.x = start[start_location][0]
    item_pose.pose.position.y = start[start_location][1]
    item_pose.pose.orientation.z = 1.0
    item_pose.pose.orientation.w = 0.0
    print('Request for pick up at start point ' + start_location)
    navigator.goToPose(item_pose)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i%5 == 0:
            print('Estimated time of arival at ' + start_location + 'to go to ' + 
            end_location + 'is {0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            )
            + ' seconds.')

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print('great success')
        final_pose = PoseStamped()
        final_pose.header.frame_id = 'map'
        final_pose.header.stamp = navigator.get_clock().now().to_msg()
        final_pose.pose.position.x = end[end_location][0]
        final_pose.pose.position.y = end[end_location][1]
        final_pose.pose.orientation.z = 1.0
        final_pose.pose.orientation.w = 0.0
        navigator.goToPose(final_pose)

    elif result == TaskResult.CANCELED:
        print('canceled')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('failed')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)

if __name__ == '__main__':
    main()

