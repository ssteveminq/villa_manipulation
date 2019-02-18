#ifndef MANIPUATION_CONTROLLER_H
#define MANIPUATION_CONTROLLER_H

#include <string>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <sensor_msgs/JointState.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <tmc_control_msgs/GripperApplyEffortGoal.h>
#include <ros/ros.h>
#include "villa_manipulation/sensor_capture.h"
#include "villa_manipulation/transform_utils.h"
#include <tf/transform_listener.h>

#define PI 3.1415
#define ACTION_TIMEOUT 10

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

class Manipulator {
private:
	ros::NodeHandle& nh_;
    ros::Subscriber joint_states_sub_;
	ros::ServiceClient srv_list_controllers_;
	Client arm_client_;
	Client base_client_;
	Client gripper_client_;
	Client head_client_;
    	OdometryCapture odom_;
        std::map<std::string, float> joint_states_;
	actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction> ac;

	void wait_for_controller(std::string name) {
        controller_manager_msgs::ListControllers list_controllers;
        bool running = false;
        while(!running && ros::ok()) {
            ros::Duration(0.1).sleep();
            if(srv_list_controllers_.call(list_controllers)) {
                for(int i = 0; i < list_controllers.response.controller.size(); i++) {
                    controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
                    if (c.name == name && c.state == "running") {
                        running = true;
                    }
                }
            }
        }
	}

	void wait_for_action(Client * client) {
        bool finished_before_timeout = client->waitForResult(ros::Duration(ACTION_TIMEOUT));
        if(finished_before_timeout) {
            actionlib::SimpleClientGoalState state = client->getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        } else {
            ROS_ERROR("Action did not finish before timeout");
        }
	}

public:

	Manipulator(ros::NodeHandle& node_handle) :
		nh_(node_handle),
	    arm_client_("/hsrb/arm_trajectory_controller/follow_joint_trajectory",true),
	    base_client_("/hsrb/omni_base_controller/follow_joint_trajectory", true),
	    gripper_client_("/hsrb/gripper_controller/follow_joint_trajectory", true),
	    head_client_("/hsrb/head_trajectory_controller/follow_joint_trajectory", true),
			ac("/hsrb/gripper_controller/grasp", true)
 {
        joint_states_sub_ = nh_.subscribe("/hsrb/robot_state/joint_states", 1, &Manipulator::updateJointStates, this);

    	srv_list_controllers_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("/hsrb/controller_manager/list_controllers");

    	ROS_INFO_STREAM("Waiting for controller manager...");
    	srv_list_controllers_.waitForExistence();
    }

	bool move_base_relative(double x_relative_to_base, double y_relative_to_base, double turn_relative_to_base, double duration);
	bool move_base_relative(double x_relative_to_base, double y_relative_to_base, double turn_relative_to_base);
    tf::Vector3 compute_new_location(tf::Vector3 rel_coords);

    control_msgs::QueryTrajectoryState get_trajectory_state(std::string name) {
        ros::ServiceClient query_client = nh_.serviceClient<control_msgs::QueryTrajectoryState>(name);
        control_msgs::QueryTrajectoryState query;
        query.request.time = ros::Time::now();

        if(!query_client.call(query)){
            ROS_ERROR("Graspserver: failed to receieve state for %s", name.c_str());
        }

        return query;
    }

    float get_joint_state(const std::string &name);

	bool set_arm_joint_state(const std::string& joint_name, double joint_value);
	bool set_arm_joint_state(const std::string& joint_name, double joint_value, double duration);

		bool rise_arm(double arm_height, double duration);
		bool rise_arm(double arm_height);

    bool move_arm(double arm_height, double arm_angle, double duration);
    bool move_arm(double arm_height, double arm_angle);

		bool move_arm_and_hand(double arm_height, double arm_angle, double hand_angle, double duration);
		bool move_arm_and_hand(double arm_height, double arm_angle, double hand_angle);

		bool grasp();

		bool move_whole_arm(double arm_lift, double arm_flex, double arm_roll, double wrist_flex, double wrist_roll, double duration);
		bool move_whole_arm(double arm_lift, double arm_flex, double arm_roll, double wrist_flex, double wrist_roll);

    bool move_gripper(double amount);
    bool move_gripper(bool open);

    bool slide_base(double x, double y, double duration);
	bool slide_base(double x, double y);
    bool turn_base(double turn_amount, double duration);
    bool turn_base(double turn_amount);

    bool move_head(double pan_angle, double tilt_angle);

    bool gripper_has_object(){ //TODO Maybe different for different robot
         if (joint_states_["hand_r_distal_joint"] > -0.01) return false;
         return true;
    };

    void updateJointStates(const sensor_msgs::JointState::ConstPtr &joint_states){
        for(unsigned int i=0; i < joint_states->name.size(); i++)
        {
            std::string joint = joint_states->name[i];
	    if(joint_states_.find(joint) != joint_states_.end()) joint_states_[joint] = joint_states->position[i];
            else joint_states_.insert(std::pair<std::string, float>(joint, joint_states->position[i]));
        }
    };
};

#endif
