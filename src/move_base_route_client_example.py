#!/usr/bin/env python3

import rospy
import actionlib
from line_planner.msg import MoveBaseRouteAction, MoveBaseRouteGoal
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

def create_goal_pose(x, y, theta):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0

    q = quaternion_from_euler(0, 0, theta)
    pose.pose.orientation = Quaternion(*q)

    return pose

rospy.init_node("move_base_route_client")

rospy.loginfo("Example node started")

client = actionlib.SimpleActionClient("move_base_route", MoveBaseRouteAction)
client.wait_for_server()

rospy.loginfo("Connected to server.")

while True:

    # Create an array of goal poses
    goals = [
        create_goal_pose(0.0, 0.0, 0),
        create_goal_pose(-7.0, 0.0, 1.57),
        create_goal_pose(0.0, 7.0, 3.14),
    ]

    route_goal = MoveBaseRouteGoal()
    route_goal.goal_poses = goals

    client.send_goal(route_goal)

    rospy.loginfo("Route sent.")

    client.wait_for_result()

    result = client.get_result()

    if result.result_status == 0:
        rospy.loginfo("All goals reached successfully")
    else:
        rospy.loginfo("Failed to reach all goals. Result: %s", result.result_text)

        