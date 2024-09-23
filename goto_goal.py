#!/usr/bin/env python

import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist


class Navigaterobot:

    def __init__(self):
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def move(self, myx, myy):
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.header.frame_id = 'map'

        goal.target_pose.pose.position.x = myx
        goal.target_pose.pose.position.y = myy
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = math.sin(-math.pi / 4)
        goal.target_pose.pose.orientation.w = math.cos(-math.pi / 4)

        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()

    #change of plan make a new function with twist only for spinning
    def spinRobo(self):
        vel_mes = Twist()
        vel_mes.linear.x = 0
        vel_mes.angular.z = 1
        for _ in range(20):
            self.velocity_pub.publish(vel_mes)
            self.rate.sleep()