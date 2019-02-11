#! /usr/bin/env python

import rospy
import actionlib

from std_msgs.msg import *
from geometry_msgs.msg import *
from hsrb_interface import Robot, exceptions, geometry
from villa_manipulation.srv import (Opendoor, OpendoorRequest, OpendoorResponse)
from villa_manipulation.srv import (Grabbag, GrabbagRequest, GrabbagResponse)
from villa_manipulation.msg import *
import math

GRAB_BAG_SRV_NAME = 'openpose/hand_positions'

good = geometry_msgs.msg.Point()
avoid = geometry_msgs.msg.Point()
goodt = geometry_msgs.msg.Point()
avoidt = geometry_msgs.msg.Point()
goodalt = geometry_msgs.msg.Point()
goodaltt = geometry_msgs.msg.Point()
avoidalt = geometry_msgs.msg.Point()
avoidaltt = geometry_msgs.msg.Point()
recog_pos = geometry_msgs.msg.PoseStamped()


class HandoverAction(object):

    def __init__(self, name, robot):
        # Init actionserver
        self._action_name = name
        self.robot = robot

        # Preparation to use robot functions
        while not rospy.is_shutdown():
            try:
                self.body = self.robot.try_get('whole_body')
                self.gripper = self.robot.try_get('gripper')
                break
            except(exceptions.ResourceNotFoundError, exceptions.RobotConnectionError) as e:
                rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

        self.grabbag_client = rospy.ServiceProxy(GRAB_BAG_SRV_NAME,Grabbag)

        self._as = actionlib.SimpleActionServer(self._action_name, villa_manipulation.msg.HandoverAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("hand detection for handover started")
        n_call = 3
        n_real_call = 0
        n_alt_call = 0
        good = geometry_msgs.msg.Point()
        avoid = geometry_msgs.msg.Point()
        try:
            for i in range(0,n_call):
                rospy.loginfo("running " + str(i) + " out of " + str(n_call) + " detections")
                rospy.wait_for_service(GRAB_BAG_SRV_NAME)
                res = self.grabbag_client()

                if (abs(res.left_hand.x+0.059)>0.03 and abs(res.right_hand.x+0.059)>0.03):
                    if res.left_hand.x < res.right_hand.x:
                        avoid = res.right_hand
                        good = res.left_hand
                    else:
                        avoid = res.left_hand
                        good = res.right_hand
                    goodt.x += good.x
                    goodt.y += good.y
                    goodt.z += good.z
                    avoidt.x += avoid.x
                    avoidt.y += avoid.y
                    avoidt.z += avoid.z
                    n_real_call += 1

                else :
                    if (res.left_hand.x > 0 and res.right_hand.x < 0):
                        goodalt = res.left_hand
                        avoidalt = res.right_hand
                    else :
                        goodalt = res.right_hand
                        avoidalt = res.left_hand
                    goodaltt.x += goodalt.x
                    goodaltt.y += goodalt.y
                    goodaltt.z += goodalt.z
                    n_alt_call += 1

            if n_real_call != 0 :
                good.x = goodt.x/n_real_call
                good.y = goodt.y/n_real_call
                good.z = goodt.z/n_real_call
                avoid.x = avoidt.x/n_real_call
                avoid.y = avoidt.y/n_real_call
                avoid.z = avoidt.z/n_real_call
            else :
                print ("using goodalt")
                good.x = goodaltt.x/n_alt_call
                good.y = goodaltt.y/n_alt_call
                good.z = goodaltt.z/n_alt_call
                avoid.x = avoidaltt.x/n_alt_call
                avoid.y = avoidaltt.y/n_alt_call
                avoid.z = avoidaltt.z/n_alt_call
                print(good)

        except rospy.ServiceException as e:
            rospy.logerr(e)
            exit(1)

        rospy.loginfo("hands detection done. proceeding to grasp object")

        # self.body.move_to_joint_positions({"arm_lift_joint": 0.4, "arm_flex_joint": -0.9,"wrist_roll_joint":-1.2, "wrist_flex_joint":0.2})
        self.open_gripper(1.0)
        recog_pos.pose.position.x = good.x - 0.2
        recog_pos.pose.position.z = good.z - 0.07
        recog_pos.pose.position.y = good.y - 0.08
        grab_pose = geometry.multiply_tuples(geometry.pose(x=recog_pos.pose.position.x,
            y=recog_pos.pose.position.y,
            z=recog_pos.pose.position.z,
            ej=math.pi/2,
            ek=math.pi/2), geometry.pose(ei=math.pi/2))
        self.body.move_end_effector_pose(grab_pose, "base_link")
        self.body.move_end_effector_by_line((0, 0, 1), 0.1)

        self.close_gripper()

        rospy.loginfo("handover action finished")
        self._as.set_succeeded()

    def open_gripper(self, to_width=1.2):
        self.gripper.command(to_width)

    def close_gripper(self, to_width=-0.01):
        # self.gripper.command(to_width)
        self.gripper.grasp(to_width)



if __name__ == '__main__':
    robot = Robot()
    # rospy.init_node('handover_action')
    rospy.loginfo("Initializing handover_action server...")
    server = HandoverAction('handover_action', robot)
    rospy.loginfo("handover_action server created")
    rospy.spin()
