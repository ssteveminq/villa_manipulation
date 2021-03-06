#! /usr/bin/env python

import rospy
import actionlib

from std_msgs.msg import *
from geometry_msgs.msg import *
from hsrb_interface import Robot, exceptions, geometry
from villa_manipulation.msg import *
import math


class GivePoseAction(object):

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

        #self.grabbag_client = rospy.ServiceProxy(GRAB_BAG_SRV_NAME,Grabbag)

        self._as = actionlib.SimpleActionServer(self._action_name, villa_manipulation.msg.HandoverAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("give pose")
        self.body.move_to_joint_positions({"arm_lift_joint":0.4, "arm_flex_joint":-0.7,"arm_roll_joint":-1.57,"wrist_roll_joint":-0.7,"wrist_flex_joint":-0.5})

        # self.body.move_to_joint_positions({"arm_lift_joint": 0.4, "arm_flex_joint": -0.9,"wrist_roll_joint":-1.2, "wrist_flex_joint":0.2})
        self.open_gripper()
        rospy.loginfo("givepose action finished")
        rospy.sleep(3)
        self.body.move_to_neutral()
        self._as.set_succeeded()

    def open_gripper(self, to_width=1.2):
        self.gripper.command(to_width)

    def close_gripper(self, to_width=-0.01):
        # self.gripper.command(to_width)
        self.gripper.grasp(to_width)



if __name__ == '__main__':
    robot = Robot()
    # rospy.init_node('handover_action')
    rospy.loginfo("Initializing givepose server...")
    server = GivePoseAction('givepose_action', robot)
    rospy.loginfo("givepose_action server created")
    rospy.spin()
