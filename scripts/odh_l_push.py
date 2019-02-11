#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import Robot
from hsrb_impedance_control_example.impedance_control_switch import ImpedanceControlSwitch
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import math
import rospy
import sys
import tf
import tf2_ros
from tf import TransformListener
import geometry_msgs.msg
from sensor_msgs.msg import JointState
import moveit_commander
import trajectory_msgs.msg
import moveit_msgs
from copy import deepcopy
from villa_navi_service.srv import GoTargetPos
from villa_navi_service.srv import GoTargetPosRequest
import controller_manager_msgs.srv
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from handle_tracking.srv import objectfinder
from villa_manipulation.srv import (Opendoor, OpendoorResponse)
import move_base_msgs.msg
from move_base_msgs.msg import MoveBaseActionGoal
from tmc_planning_msgs.srv import PlanWithTsrConstraints
from tmc_planning_msgs.srv import PlanWithTsrConstraintsRequest
from tmc_planning_msgs.msg import TaskSpaceRegion
from tmc_manipulation_msgs.msg import (
	BaseMovementType,
	ArmManipulationErrorCodes
)
from geometry_msgs.msg import WrenchStamped
_CONNECTION_TIMEOUT = 10.0
_TF_TIMEOUT = 3.5
_ODOM_TF = 'map'
# _ORIGIN_TF ='map'
_ORIGIN_TF ='map'
_BASE_TF = 'base_link'
_ROBOT_TF = 'base_footprint'
_HAND_TF = 'hand_palm_link'

_USE_JOINTS = (
	'wrist_flex_joint',
	'wrist_roll_joint',
	'arm_roll_joint',
	'arm_flex_joint',
	'arm_lift_joint'
)
_PLANNING_MAX_ITERATION = 10000
_PLANNING_GOAL_GENERATION = 0.3
_PLANNING_GOAL_DEVIATION = 0.3

HANDLE_POS = (5.1, 0.51, 0.947)
RECOG_HANDLE_POS = (0.0, 0.0, 0.0)
HANDLE_TO_DOOR_HINGE_POS = 0.61
HANDLE_TO_HANDLE_HINGE_POS = -0.06
HANDLE_TO_HAND_POS = 0.067
HANDLE_GOAL_OFFSET = 0.5
HANDLE_TO_HAND_POS_X = 0.1
HANDLE_TO_HAND_POS_Y=0.012


recog_pos = geometry_msgs.msg.PoseStamped()
recog_pos2 = geometry_msgs.msg.PoseStamped()
hand_pos = geometry_msgs.msg.PoseStamped()

latest_positions = None


class setRobot:
	def __init__(self):
		self.rob = Robot()
		self.whole_body = self.rob.get('whole_body')
		self.omni_base = self.rob.get('omni_base')
		self.gripper = self.rob.get('gripper')
		
		# collision_world = self.rob.try_get('global_collision_world')
		# self.whole_body.collision_world = collision_world

		self.whole_body.end_effector_frame = u'hand_palm_link'
		
class ForceSensorCapture(object):
    """Subscribe and hold force sensor data"""

    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0

        # Subscribe force torque sensor data from HSRB
        ft_sensor_topic = '/hsrb/wrist_wrench/compensated'
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)
        # Wait for connection
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped,
                                   timeout=_CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def __ft_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z

def publish_arm(lift, flex,roll,wrist_flex,wrist_roll):
	traj = JointTrajectory()
	# This controller requires that all joints have values
	traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
						"arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
	p = JointTrajectoryPoint()
	current_positions = [latest_positions[name] for name in traj.joint_names]
	current_positions[0] = lift
	current_positions[1] = flex
	current_positions[2] = roll
	current_positions[3] = wrist_flex
	current_positions[4] = wrist_roll
	p.positions = current_positions
	p.velocities = [0, 0, 0, 0, 0]
	p.time_from_start = rospy.Time(3)
	traj.points = [p]

	armPub.publish(traj)

def call_tsr_plan_service(whole_body, constraint_tsrs, goal_tsrs):
	odom_to_robot_pose = geometry.tuples_to_pose(get_relative_tuples(_ORIGIN_TF,
																	 _ROBOT_TF))
	req = PlanWithTsrConstraintsRequest()
	req.base_movement_type.val = BaseMovementType.PLANAR
	req.origin_to_basejoint = odom_to_robot_pose
	req.initial_joint_state = whole_body._get_joint_state()
	req.use_joints = _USE_JOINTS
	req.probability_goal_generate = _PLANNING_GOAL_GENERATION
	req.timeout = rospy.Duration(whole_body._planning_timeout)
	req.max_iteration = _PLANNING_MAX_ITERATION
	req.uniform_bound_sampling = False
	req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION
	req.weighted_joints = ['_linear_base', '_rotational_base']
	req.weight = [whole_body._linear_weight, whole_body._angular_weight]
	if whole_body._collision_world is not None:
		req.environment_before_planning = whole_body._collision_world.snapshot(_ORIGIN_TF)
	req.constraint_tsrs = constraint_tsrs
	req.goal_tsrs = goal_tsrs
	plan_service = rospy.ServiceProxy('plan_with_constraints',
											PlanWithTsrConstraints)
	res = plan_service.call(req)
	return res


def get_relative_tuples(base_frame, target_frame):

	while not rospy.is_shutdown():
		try:
			trans = whole_body._tf2_buffer.lookup_transform(base_frame,
								target_frame,
								rospy.Time(0),
								rospy.Duration(_TF_TIMEOUT)).transform
			break
		except ExtrapolationException as e:
			continue
			
	tuples = geometry.transform_to_tuples(trans)
	return tuples

def grasp_point_callback(msg):
	recog_pos.pose.position.x=msg.pose.position.x
	recog_pos.pose.position.y=msg.pose.position.y
	recog_pos.pose.position.z=msg.pose.position.z
	# dkldsaflk;print recog_pos.pose.position

def grasp_point_client():
	try:
		print "calling grasping point service"
		localize_handle = rospy.ServiceProxy('/track_handle',objectfinder)
		global Is_found
		Is_found=False
		while(Is_found==False):
			Response=localize_handle()
			Is_found =Response.handle_is_found
			print Response.best_grasp_pose
			print Is_found
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
		print "service grasp point failed"
		
def navi_service_client(x_,y_,theta_):
	try:
		print "calling navi service"
		nav_srv_client = rospy.ServiceProxy("navi_go_base", GoTargetPos)
		nav_req=GoTargetPosRequest()
		nav_req.x_from_map=x_
		nav_req.y_from_map=y_
		nav_req.theta_from_map=theta_
		nav_srv_client(nav_req)

	except rospy.ServiceException, e:
		print "navi service failed"


def joint_states_callback(msg):
	global latest_positions
	positions = {}
	for name, i in zip(msg.name, range(len(msg.name))):
		positions[name] = msg.position[i]
	latest_positions = positions

def opendoor(req):
	# main(whole_body,  gripper,wrist_wrench)
	frame = req.handle_pose.header.frame_id
	hanlde_pos = req.handle_pose.pose
	# hanlde_pos=geometry_msgs.msg.PoseStamped()
	res = OpendoorResponse()
	cli = actionlib.SimpleActionClient('/hsrb/omni_base_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
	vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)
	armPub = rospy.Publisher('/hsrb/arm_trajectory_controller/command', JointTrajectory, queue_size=1)

	robot = hsrb_interface.Robot()
	whole_body = robot.get('whole_body')
	gripper = robot.get('gripper')
	wrist_wrench = robot.get('wrist_wrench')
	base=robot.get('omni_base')
	start_theta=base.pose[2]
	# with hsrb_interface.Robot() as robot:
			# whole_body = robot.get('whole_body')
			# gripper = robot.get('gripper')
			# wrist_wrench = robot.get('wrist_wrench')

	try: 
		# Open the gripper 
		whole_body.move_to_neutral()
		grasp_point_client()
		global recog_pos
		global is_found
		print recog_pos.pose.position

		print("Opening the gripper")
		whole_body.move_to_neutral()
		rospy.sleep(2.5)
		switch = ImpedanceControlSwitch() 
		gripper.command(1.0)
		# Approach to the door

		listener = tf.TransformListener()
		listener.waitForTransform(_ORIGIN_TF,_BASE_TF, rospy.Time(), rospy.Duration(4.0))
		# translation,rot = listener.lookupTransform(_BASE_TF, _ORIGIN_TF, rospy.Time())	
		recog_pos.header.frame_id=_ORIGIN_TF
		recog_pos2 = listener.transformPose(_BASE_TF, recog_pos)		

		print("Approach to the door")
		grab_pose = geometry.multiply_tuples(geometry.pose(x=recog_pos2.pose.position.x-HANDLE_TO_HAND_POS_X,
			y=recog_pos2.pose.position.y,
			z=recog_pos2.pose.position.z,
			ej=math.pi/2),
			geometry.pose(ek=math.pi/2)) 
			# 	grab_pose = geometry.multiply_tuples(geometry.pose(x=recog_pos2.pose.position.x-HANDLE_TO_HAND_POS_X/2,
			# y=recog_pos2.pose.position.y-HANDLE_TO_HAND_POS_X/2,
			# z=recog_pos2.pose.position.z,
			# ej=0,
			# ek=math.pi/2), geometry.pose(ei=math.pi/2)) 
		# grab_pose = geometry.multiply_tuples(geometry.pose(x=recog_pos.pose.position.x-translation[0]-HANDLE_TO_HAND_POS_X,
		# 			y=recog_pos.pose.position.y-translation[1],
		# 			z=recog_pos.pose.position.z-translation[2],
		# 			ej=math.pi/2),
		# 			geometry.pose(ek=0.0))
		# 			# geometry.pose(ek=math.pi/2))
		
		whole_body.move_end_effector_pose(grab_pose, _BASE_TF)
		whole_body.move_end_effector_by_line((0, 0, 1), 0.06)
		# Close the gripper
		wrist_wrench.reset()
		switch.activate("grasping")
		gripper.grasp(-0.01)
		rospy.sleep(1.0)
		switch.inactivate()
		# rospy.sleep(100)
	# Rotate the handle
		whole_body.impedance_config = 'grasping'
		

         whole_body.move_end_effector_by_line((0,1,0),-0.015)
        rospy.sleep(1)
        whole_body.move_end_effector_by_line((0,0,1),0.06)
        rospy.sleep(1)	
        whole_body.move_end_effector_by_line((0,1,0),0.015)
		gripper.command(1.0)


                # traj = JointTrajectory()

		# # This controller requires that all joints have values
		# traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
		# 										"arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
		# p = JointTrajectoryPoint()
		# current_positions = [latest_positions[name] for name in traj.joint_names]
		# current_positions[0] = latest_positions["arm_lift_joint"]-0.0675
		# current_positions[1] = latest_positions["arm_flex_joint"]-0.02
		# current_positions[2] = latest_positions["arm_roll_joint"]
		# current_positions[3] = latest_positions["wrist_flex_joint"]
		# current_positions[4] = latest_positions["wrist_roll_joint"]-0.65
		# p.positions = current_positions
		# p.velocities = [0, 0, 0, 0, 0]
		# p.time_from_start = rospy.Time(3)
		# traj.points = [p]

		# armPub.publish(traj)
		# rospy.sleep(3.0)
		# print("finishing rotating handle") 
		# ## Move by End-effector line
		# whole_body.impedance_config = 'compliance_hard'
		# whole_body.move_end_effector_by_line((0.0,0.0,1), 0.45)
		print("pull the door") 
		# Move base with linear & Angular motion
		# hand_pos.header.frame_id=u'hand_palm_link'
		print("test 1") 

		tw = geometry_msgs.msg.Twist()
		tw.linear.x = 0.3
		tw.linear.y = -0.45
		vel_pub.publish(tw)
		
		rospy.sleep(2.0)
		print("test2 ") 

		tw = geometry_msgs.msg.Twist()
		tw.linear.x =0.3
		vel_pub.publish(tw)
		rospy.sleep(1.0)

		tw = geometry_msgs.msg.Twist()
		tw.linear.x =0.3
		vel_pub.publish(tw)
	#############keep
		rospy.sleep(1.0)
	
		tw = geometry_msgs.msg.Twist()
		tw.linear.x =0.3
		vel_pub.publish(tw)

		whole_body.move_to_neutral()
		res.success = True 

	except Exception as e:
		rospy.logerr(e)
		print "Failed to open door"
		res.success = False
	return res

def opendoor_server():
		# rospy.init_node('opendoor_server_node')
		rospy.Subscriber("hsrb/joint_states",JointState,joint_states_callback)
		s = rospy.Service('open_l_push_service', Opendoor, opendoor)
		rospy.spin()


if __name__=='__main__':
	global robot
	print("Initialize Robot")
	#this function has node so you don't have to initialize node
	rrobot = setRobot()
	opendoor_server()
	print("Start service")
