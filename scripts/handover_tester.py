#!/usr/bin/env python

import rospy
import actionlib
import geometry_msgs
from villa_manipulation.msg import HandoverAction, HandoverGoal

rospy.init_node('handoverClient')

handoverClient = actionlib.SimpleActionClient('handover_action', HandoverAction)
rospy.loginfo("Waiting for server")
handoverClient.wait_for_server()

rospy.loginfo("Sending goal")
goal = HandoverGoal()
handoverClient.send_goal(goal)
handoverClient.wait_for_result(rospy.Duration.from_sec(45.0))
