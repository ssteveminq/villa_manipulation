#! /usr/bin/env python

import rospy
import actionlib

import moveit_commander
from std_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import *
from hsrb_interface import Robot, exceptions, geometry
from villa_manipulation.msg import *
import math

class MoveIt_IK_Action(object):

    def __init__(self, name):
        # Init actionserver
        self._action_name = name
        # self.robot = robot

        moveit_commander.roscpp_initialize(sys.argv)

        arm = moveit_commander.MoveGroupCommander('arm')
        head = moveit_commander.MoveGroupCommander('head')
        whole_body = moveit_commander.MoveGroupCommander('whole_body')
        whole_body_weighted \
            = moveit_commander.MoveGroupCommander('whole_body_weighted')
        whole_body_light \
            = moveit_commander.MoveGroupCommander('whole_body_light')


        # Preparation to use robot functions
        # while not rospy.is_shutdown():
            # try:
                # self.body = self.robot.try_get('whole_body')
                # self.gripper = self.robot.try_get('gripper')
                # break
            # except(exceptions.ResourceNotFoundError, exceptions.RobotConnectionError) as e:
                # rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

        #self.grabbag_client = rospy.ServiceProxy(GRAB_BAG_SRV_NAME,Grabbag)

        self._as = actionlib.SimpleActionServer(self._action_name, villa_manipulation.msg.IK_Action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        self.desired_pan=0.0
        self.desired_tilt=0.0
        # rospy.Subscriber('desired_head_pan',Float32,self.cmd_cb)
        # rospy.Subscriber('/hsrb/joint_states', JointState, self.states_cb)

    def cmd_cb(self,msg):
        #clatest_positions
        cmd= msg.data
        # pan =1.0*cmd+self.latest_positions["head_pan_joint"]
        self.desired_pan=cmd
        self.desired_tilt=0.1
 

    def execute_cb(self, goal):
        # rospy.loginfo("head_tracking pose- pan/ tilt: %.2lf, %2lf", self.desired_pan, self.desired_tilt)
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "hand_palm_link"
        p.pose.position.z = 0.4
        p.pose.orientation.w = 1
        whole_body.set_joint_value_target(p)
        whole_body.go()
        rospy.sleep(0.1)
        # self.body.move_to_joint_positions({"head_pan_joint":self.desired_pan, "head_tilt_joint":self.desired_tilt})
        # rospy.sleep(0.5)
        self._as.set_succeeded()

    def states_cb(self,msg):
        positions = {}
        for name, i in zip(msg.name, range(len(msg.name))):
            positions[name] = msg.position[i]
        self.latest_positions = positions

if __name__ == '__main__':
    rospy.init_node('moveit_ik', anonymous=True)
    server = MoveIt_IK_Action('moveit_ik')
    rospy.loginfo("move_it_ik_action_server")
    rospy.spin()
