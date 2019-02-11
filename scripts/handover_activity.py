#!/usr/bin/env python
from __future__ import print_function
from villa_manipulation.srv import (Grabbag, GrabbagResponse)
import geometry_msgs.msg
from hsrb_impedance_control_example.impedance_control_switch import ImpedanceControlSwitch
import math

import control_msgs.msg
import tf
import tf2_ros
from tf import TransformListener
import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import Robot

import rospy
import sys
from villa_manipulation.srv import GetHandPositions
from handle_tracking.srv import objectfinder
_ORIGIN_TF ='map'
_BASE_TF = 'base_link'
# from openpose_ros_wrapper_msgs.msg import Persons
# from openpose_ros_wrapper_msgs.msg import PersonDetection
# from openpose_ros_wrapper_msgs.msg import BodyPartDetection
left_pos = geometry_msgs.msg.Point()
left_pos_f = geometry_msgs.msg.Point()
right_pos = geometry_msgs.msg.Point()
right_pos_f = geometry_msgs.msg.Point()
recog_pos = geometry_msgs.msg.PoseStamped()
recog_pos2 = geometry_msgs.msg.PoseStamped()

HANDLE_TO_HAND_POS_X = 0.1

hand_service = rospy.ServiceProxy("/openpose/hand_positions",GetHandPositions)
localize_handle = rospy.ServiceProxy('/track_handle',objectfinder)

class setRobot:
	def __init__(self):
		s = rospy.Service('grab_the_bag', Grabbag, go_to)
		self.rob = Robot()
		self.whole_body = self.rob.get('whole_body')
		self.omni_base = self.rob.get('omni_base')
		self.gripper = self.rob.get('gripper')
		
		# collision_world = self.rob.try_get('global_collision_world')
		# self.whole_body.collision_world = collision_world

		self.whole_body.end_effector_frame = u'hand_palm_link'
		rospy.spin()
def go_to(req):
	print("starting go_to")
	# hand_service = rospy.ServiceProxy("/openpose/hand_positions",GetHandPositions)
	n_call = 3
	global left_pos_f
	global right_pos_f
	left_pos_f.x=0
	left_pos_f.y=0
	left_pos_f.z=0
	right_pos_f.x=0
	right_pos_f.y=0
	right_pos_f.z=0
	for i in range(0,n_call):
		rospy.sleep(2)
		rospy.wait_for_service("/openpose/hand_positions")
		res1 = hand_service.call()
		print("Received left and right :")
		print(res1.left_hand)
		print(res1.right_hand)
		left_pos = res1.left_hand
		right_pos = res1.right_hand
		
		if (abs(left_pos.x+0.059)>0.04 and abs(left_pos.x+0.059)>0.04):
			print(abs(left_pos.x+0.059))
			print(abs(right_pos.x+0.059))
			left_pos_f.x += left_pos.x
			left_pos_f.y += left_pos.y 
			left_pos_f.z += left_pos.z 
			right_pos_f.x += right_pos.x
			right_pos_f.y += right_pos.y
			right_pos_f.z += right_pos.z 

	if left_pos.x < right_pos.x:
		avoid = right_pos_f
		good = left_pos_f
	else:
		avoid = left_pos_f
		good = right_pos_f
	good.x = good.x/n_call
	good.y = good.y/n_call
	good.z = good.z/n_call
	avoid.x = good.x/n_call
	avoid.y = good.y/n_call
	avoid.z = good.z/n_call

	vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)
	print("accessing robot")
	robot = hsrb_interface.Robot()
	whole_body = robot.get('whole_body')
	gripper = robot.get('gripper')
	wrist_wrench = robot.get('wrist_wrench')
	base=robot.get('omni_base')	
	print("accessing robot complete")
	try:
		# whole_body.move_to_neutral()
		# print("looking for grasping point")
		# grasp_point_client(avoid,good)
		# global recog_pos
		# global is_found
		# print (recog_pos.pose.position)	

		print("Opening the gripper")
		whole_body.move_to_neutral()
		# rospy.sleep(2.5)
		switch = ImpedanceControlSwitch() 
		gripper.command(1.0)
		# Approach to the door

		# listener = tf.TransformListener()
		# listener.waitForTransform(_ORIGIN_TF,_BASE_TF, rospy.Time(), rospy.Duration(4.0))
		# # translation,rot = listener.lookupTransform(_BASE_TF, _ORIGIN_TF, rospy.Time())	
		# recog_pos.header.frame_id=_ORIGIN_TF
		# recog_pos2 = listener.transformPose(_BASE_TF, recog_pos)		

		recog_pos2.pose.position.x = good.x - 0.05
		recog_pos2.pose.position.z = good.z - 0.07
		recog_pos2.pose.position.y = good.y - 0.1
		grab_pose = geometry.multiply_tuples(geometry.pose(x=recog_pos2.pose.position.x-HANDLE_TO_HAND_POS_X,
			y=recog_pos2.pose.position.y,
			z=recog_pos2.pose.position.z,
			ej=math.pi/2,
			ek=math.pi/2), geometry.pose(ei=math.pi/2)) 
		
		whole_body.move_end_effector_pose(grab_pose, _BASE_TF)
		whole_body.move_end_effector_by_line((0, 0, 1), 0.1)
		# Close the gripper
		wrist_wrench.reset()
		switch.activate("grasping")
		gripper.grasp(-0.01)
		rospy.sleep(1.0)
		switch.inactivate()
		whole_body.impedance_config = 'grasping'
		tw = geometry_msgs.msg.Twist()
		tw.linear.x =-0.3
		vel_pub.publish(tw)
		rospy.sleep(2)
		whole_body.move_to_neutral()
		gripper.command(1.0)

	except Exception as e:
		rospy.logerr(e)
		print ("Failed to open door")
		# res.success = False
	res = True
	return res

def grabbag_server():
	# rospy.init_node('opendoor_server_node')
	# rospy.Subscriber("hsrb/joint_states",JointState,joint_states_callback)
	s = rospy.Service('grab_the_bag', Grabbag, go_to)
	rospy.spin()

def grasp_point_callback(msg):
	#recog_pos.pose.position.x=msg.poses[0].pose.position.x
	#recog_pos.pose.position.y=msg.pose[0].pose.position.y
	#recog_pos.pose.position.z=msg.pose[0].pose.position.z
	recog_pos.pose.position.x=msg.poses[0].position.x
	recog_pos.pose.position.y=msg.poses[0].position.y
	recog_pos.pose.position.z=msg.poses[0].position.z
	# dkldsaflk;print recog_pos.pose.position

def grasp_point_client(avoid,good):
	try:
		print ("calling grasping point service")
		# objectfinder.active_bag_grasping = True
		# objectfinder.radius_hand = 0.06
		# objectfinder.avoid_hand = avoid

		global Is_found
		Is_found=False
		rospy.wait_for_service("/track_handle")
		Response=localize_handle(False,0.06,True,good,avoid)
		Is_found =Response.handle_is_found
		print (Response.best_grasp_pose)
		print (Is_found)
		if Is_found:
			global recog_pos
			# recog_pos=Response.best_grasp_pose
			recog_pos.pose.position.x=Response.best_grasp_pose.pose.position.x
			recog_pos.pose.position.y=Response.best_grasp_pose.pose.position.y
			recog_pos.pose.position.z=Response.best_grasp_pose.pose.position.z
			recog_pos.pose.orientation.x=Response.best_grasp_pose.pose.orientation.x
			recog_pos.pose.orientation.y=Response.best_grasp_pose.pose.orientation.y
			recog_pos.pose.orientation.z=Response.best_grasp_pose.pose.orientation.z
			recog_pos.pose.orientation.w=Response.best_grasp_pose.pose.orientation.w
				
	except rospy.ServiceException, e:
		print ("service grasp point failed")


if __name__ == '__main__':

	robot = setRobot()
	grabbag_server()

