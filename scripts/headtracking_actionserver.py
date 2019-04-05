#! /usr/bin/env python

import rospy
import actionlib

from std_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import *
from hsrb_interface import Robot, exceptions, geometry
from villa_manipulation.msg import *
import math


class HeadTrackAction(object):

    def __init__(self, name, robot):
        # Init actionserver
        self._action_name = name
        self.robot = robot

        # Preparation to use robot functions
        while not rospy.is_shutdown():
            try:
                self.body = self.robot.try_get('whole_body')
                # self.gripper = self.robot.try_get('gripper')
                break
            except(exceptions.ResourceNotFoundError, exceptions.RobotConnectionError) as e:
                rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

        self._as = actionlib.SimpleActionServer(self._action_name, villa_manipulation.msg.HeadTrackingAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)
        self.tw = geometry_msgs.msg.Twist()
        self.desired_pan=0.0
        self.desired_tilt=0.0
        rospy.Subscriber('desired_head_pan',Float32,self.cmd_cb)
        rospy.Subscriber('desired_base_forward',Int8,self.base_cmd_cb)
        rospy.Subscriber('/hsrb/joint_states', JointState, self.states_cb)

    def cmd_cb(self,msg):
        #clatest_positions
        cmd= msg.data
        # pan =1.0*cmd+self.latest_positions["head_pan_joint"]
        self.desired_pan=cmd
        self.desired_tilt=0.1
 
    def base_cmd_cb(self,msg):
        #clatest_positions
        cmd= msg.data
        if cmd ==1:
            self.tw.linear.x=0.25
        elif cmd ==2:
            self.tw.linear.x=-0.25
        else:
            self.tw.linear.x=0
 

    def execute_cb(self, goal):
        rospy.loginfo("head_tracking pose- pan/ tilt: %.2lf, %2lf", self.desired_pan, self.desired_tilt)
        #pbulish_head_command
        self.body.move_to_joint_positions({"head_pan_joint":self.desired_pan, "head_tilt_joint":self.desired_tilt})
        #publish base command
        self.vel_pub.publish(self.tw)
        self.tw=geometry_msgs.msg.Twist()
        # rospy.sleep(0.5)
        self._as.set_succeeded()

    def states_cb(self,msg):
        positions = {}
        for name, i in zip(msg.name, range(len(msg.name))):
            positions[name] = msg.position[i]
        self.latest_positions = positions



if __name__ == '__main__':
    robot = Robot()
    server = HeadTrackAction('headtracking_action', robot)
    rospy.loginfo("headtracking_action server created")
    rospy.spin()
