#! /usr/bin/env python

import rospy
import actionlib
import tf
import tf2_ros
from tf import TransformListener
from std_msgs.msg import *
from geometry_msgs.msg import *
from hsrb_interface import Robot, exceptions, geometry
from villa_manipulation.msg import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import math

_ORIGIN_TF ='map'
_BASE_TF = 'base_link'
_ARM_TF = 'arm_lift_link'
_HAND_TF = 'hand_palm_link'
ARM_LINK_OFFSET_Z=0.3
ObJ_LIFT_OFFSET_Z=0.05
MAX_ARM_LIFT=0.67 #joint limit of arm lift joint 
MAX_HEIGHT_SHOULDER=MAX_ARM_LIFT+ARM_LINK_OFFSET_Z #
ARM_LENGTH=0.3#
BASE_APPROACH_OFFSET_X=0.18
# BASE_APPROACH_OFFSET_X=0.35
ARM_OFFSET_X=0.05

class PutdownPoseAction(object):

    def __init__(self, name, robot):
        # Init actionserver
        self._action_name = name
        self.robot = robot
        self.force_sensor= WrenchStamped()
        self.last_force=WrenchStamped()
        self.Touch_tabletop=False


        jointstates_topic='hsrb/joint_states'
	rospy.Subscriber(jointstates_topic, JointState, self.joint_state_Cb)

        force_sensor_topic='hsrb/wrist_wrench/raw'
	rospy.Subscriber(force_sensor_topic, WrenchStamped, self.force_sensor_Cb)

        # Preparation to use robot functions
        while not rospy.is_shutdown():
            try:
                self.body = self.robot.try_get('whole_body')
                self.gripper = self.robot.try_get('gripper')
                self.base=self.robot.try_get('omni_base')
                break
            except(exceptions.ResourceNotFoundError, exceptions.RobotConnectionError) as e:
                rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

        #self.grabbag_client = rospy.ServiceProxy(GRAB_BAG_SRV_NAME,Grabbag)
        # self.initial_weight=self.force_senor.wrench.force.x
        # self.test_armmotion()
        self.setdown();

        # self._as = actionlib.SimpleActionServer(self._action_name, villa_manipulation.msg.HandoverAction, execute_cb=self.execute_cb, auto_start = False)
        # self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("putdown pose")
        # self.body.move_to_joint_positions({"arm_lift_joint":0.655, "arm_flex_joint":-1.45,"arm_roll_joint":1.49,"wrist_roll_joint":-1.45,"wrist_flex_joint":0.0})
        self.body.move_to_joint_positions({"arm_lift_joint":0.655, "arm_flex_joint":-1.45,"arm_roll_joint":1.49,"wrist_roll_joint":-1.45,"wrist_flex_joint":0.0})
        rospy.sleep(4)
        self.body.move_end_effector_by_line((0,0,1),0.5)
        rospy.sleep(2)
        self.open_gripper()
        rospy.sleep(3)
        self.body.move_end_effector_by_line((0,0,1),-0.6)
        #self.base.go_rel(-1.0,0.0,0)

        #self.close_gripper()
        #rospy.sleep(4)
        #self.body.move_to_joint_positions({"arm_lift_joint": 0.0, "arm_flex_joint": -0.0,"arm_roll_joint": 2.6,"wrist_roll_joint":1.4, "wrist_flex_joint":-1.45})
        rospy.sleep(2)
        rospy.loginfo("putdown action finished")
        self.body.move_to_neutral()
        rospy.sleep(2)
        self._as.set_succeeded()

    def test_armmotion(self):
        # transvec_from_position_arm = 
        rospy.sleep(5)
        # self.close_gripper()
        #target_pose is represented w.r.t map
        target_pose=PoseStamped()
        target_pose.pose.position.x=4.65
        target_pose.pose.position.y=1.6
        target_pose.pose.position.z=1.04
        target_pose.header.frame_id=_ORIGIN_TF

	listener = tf.TransformListener()
	listener.waitForTransform(_ORIGIN_TF,_BASE_TF, rospy.Time(), rospy.Duration(2.0))
        target_pose_base= listener.transformPose(_BASE_TF, target_pose)


        rospy.sleep(2)
	listener = tf.TransformListener()
	listener.waitForTransform(_ARM_TF,_BASE_TF, rospy.Time(), rospy.Duration(2.0))

        # target_pose.header.frame_id=_BASE_TF

        if  target_pose_base.pose.position.z < 0.1:
            rospy.loginfo("the target height is too low")
        elif  target_pose_base.pose.position.z < MAX_HEIGHT_SHOULDER:
            target_pose_arm= listener.transformPose(_ARM_TF, target_pose_base)
            self.body.move_to_joint_positions({"arm_lift_joint":target_pose_arm.pose.position.z, "arm_flex_joint":-math.pi/2, "arm_roll_joint": 0.0,"wrist_roll_joint":0.0, "wrist_flex_joint":0.0})
        elif target_pose_base.pose.position.z < MAX_HEIGHT_SHOULDER+ARM_LENGTH:
            shoulder_angle = math.asin((target_pose_base.pose.position.z-MAX_HEIGHT_SHOULDER)/ARM_LENGTH)
            self.body.move_to_joint_positions({"arm_lift_joint":MAX_ARM_LIFT, "arm_flex_joint":-math.pi/2+shoulder_angle, "arm_roll_joint": 0.0,"wrist_roll_joint":0.0, "wrist_flex_joint":-shoulder_angle})
        else:
            rospy.loginfo("the target height is too high")


	listener.waitForTransform(_HAND_TF ,_BASE_TF, rospy.Time(), rospy.Duration(4.0))

        object_hand_offset= listener.transformPose(_HAND_TF ,target_pose_base)
        print object_hand_offset

        object_hand_offset_x = object_hand_offset.pose.position.x;
        object_hand_offset_y = object_hand_offset.pose.position.y;
        object_hand_offset_z = object_hand_offset.pose.position.z;

        print object_hand_offset_y
        self.base.go_rel(0.0,-object_hand_offset_y,0)

        desired_base_distance = object_hand_offset_z-BASE_APPROACH_OFFSET_X
        print desired_base_distance 

        if desired_base_distance>0:
            self.base.go_rel(desired_base_distance,0.0,0)

        # current_weight = abs(self.force_sensor.wrench.force.x)
        # //double threshold = 0.4 * current_weight + 0.6 * initial_weight;
        # double threshold = 1.2*current_weight;
        # double current_height;

        # std::cout << "Current weight: " << current_weight << std::endl;
        # std::cout << "Resting weight: " << initial_weight << std::endl;
        # std::cout << "Threshold:      " << threshold << std::endl;


        # while self.Touch_tabletop == False:
            # current_height = manipulator.get_trajectory_state("/hsrb/arm_trajectory_controller/query_state").response.position[0];
            # std::cout << "Current weight: " << current_weight << std::endl;
            # self.body.move_to_joint_positions({"arm_lift_joint":self.cur_arm_lift-0.01})
            # manipulator.rise_arm(current_height - 0.01, 1.0);
            # current_weight = abs(getCurrentWeight());
            # rospy.loginfo("decreasing")
        
        # rospy.sleep(1.0)
        # ros::Duration(1.5).sleep();

        # double horizontalDistance = obj_base_link[1] - hand_base_link[1];
        # double forwardDistance = obj_base_link[0] - hand_base_link[0];


        # print target_pose_arm

    def setdown(self):
        # self.close_gripper();
        rospy.sleep(2)
        # self.body.impedance_config='compliance_hard'
        # rospy.sleep(2)

        while self.Touch_tabletop == False & ((self.cur_arm_lift+ ARM_LINK_OFFSET_Z) > target_pose.pose.position.z-0.02):
            # current_height = manipulator.get_trajectory_state("/hsrb/arm_trajectory_controller/query_state").response.position[0];
            # std::cout << "Current weight: " << current_weight << std::endl;
            self.body.move_to_joint_positions({"arm_lift_joint":self.cur_arm_lift-0.01})
            # manipulator.rise_arm(current_height - 0.01, 1.0);
            # current_weight = abs(getCurrentWeight());
            rospy.loginfo("decreasing")
            rospy.sleep(0.1)
        rospy.loginfo("--------------finished")
        

    def joint_state_Cb(self, msg):
        self.cur_arm_flex=msg.position[0]
        self.cur_arm_lift=msg.position[1]
        self.cur_arm_roll=msg.position[2]

    def calculate_variance(self, pre_force,cur_force):
        force_var=0.0;
        force_var+=math.pow(pre_force.wrench.force.x-cur_force.wrench.force.x,2)
        force_var+=math.pow(pre_force.wrench.force.y-cur_force.wrench.force.y,2)
        force_var+=math.pow(pre_force.wrench.force.z-cur_force.wrench.force.z,2)
        force_var=math.sqrt(force_var)
        # rospy.loginfo("force change: %.2lf", force_var)

        cur_time = rospy.get_time()
        duration = self.force_time = rospy.get_time()

        if force_var>0.7:
            self.Touch_tabletop=True
            self.force_time =rospy.get_time()
        else:
            if duration <2.0:
                return;
            else:
                self.Touch_tabletop=False

        # rospy.loginfo("force sensing %d", self.Touch_tabletop)


    def force_sensor_Cb(self,msg):
        self.force_sensor=msg
        self.calculate_variance(msg,self.last_force)
        self.last_force=msg
        
        # rospy.loginfo("forcesensor callback")
        # rospy.loginfo("x: %.2lf, y: %.2lf, z: %.2lf", msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z)

    def open_gripper(self, to_width=1.2):
        self.gripper.command(to_width)

    def close_gripper(self, to_width=-0.01):
        # self.gripper.command(to_width)
        self.gripper.grasp(to_width)



if __name__ == '__main__':
    robot = Robot()
    # rospy.init_node('handover_action')
    rospy.loginfo("Initializing putdownpose server...")
    server = PutdownPoseAction('putdown_action', robot)
    rospy.loginfo("putdownpose_action server created")
    rospy.spin()
