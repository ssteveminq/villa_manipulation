#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
from hsrb_interface import geometry
import rospy
import sys
import tf
import actionlib
from tmc_geometry_msgs.msg import Point2DStamped
import controller_manager_msgs.srv
from sensor_msgs.msg import JointState
import trajectory_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from tmc_perspective_transformer.srv import (InversePerspectiveTransform,
                                             InversePerspectiveTransformRequest)
from villa_manipulation.srv import (Opendoor, OpendoorRequest, OpendoorResponse)
from villa_manipulation.msg import *

_CONNECTION_TIMEOUT = 10.0
_INV_PERSPECTIVE_TRANSFORM_SRV_NAME = '/inverse_perspective_transform'
_TARGET_POINT_X = 290.0
_TARGET_POINT_Y = 290.0
_VIEW_POSITIONS = {'head_pan_joint': -1.0,
                   'head_tilt_joint': -0.8,
                   'arm_flex_joint': -0.5}

latest_positions = None
recog_pos = geometry_msgs.msg.PoseStamped()

def mains():
    # Initialize
    client = actionlib.SimpleActionClient('putdown_action',villa_manipulation.msg.ForcePutDownAction)
    client.wait_for_server()
    goal = villa_manipulation.msg.ForcePutDownGoal()
    goal.place_pose =PoseStamped()
    goal.place_pose.pose.position.x=1.5
    goal.place_pose.pose.position.y=0.45
    goal.place_pose.pose.position.z=0.78
    goal.place_pose.header.frame_id='map'
 
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(25.0))
    result_state= client.get_state()
    rospy.loginfo("cur state: %d", result_state)
    
    rospy.loginfo("start action")
        
if __name__ == '__main__':
    rospy.init_node('putdown_client')
    mains()
