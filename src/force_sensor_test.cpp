/// @brief Speak Object Weight Sample
/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <cstdlib>
#include <math.h>
#include <string>

#include <boost/format.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <tmc_control_msgs/GripperApplyEffortGoal.h>
#include <tmc_manipulation_msgs/SafeJointChange.h>
#include <tmc_msgs/TalkRequestAction.h>
#include <tmc_msgs/TalkRequestGoal.h>
#include <tmc_msgs/Voice.h>

geometry_msgs::Vector3 force_data_;

void force_callback(const geometry_msgs::WrenchStampedConstPtr& data) {
	force_data_ = data->wrench.force;
	ROS_INFO_STREAM("Force data: " << force_data_.x << " " << force_data_.y << " " << force_data_.z);
	ros::Duration(0.5).sleep();
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "force_sensor_test");

	ros::NodeHandle nh;
	std::string ft_sensor_topic = "/hsrb/wrist_wrench/raw";
	ros::Subscriber wrist_wrench_sub_ = nh.subscribe(ft_sensor_topic, 1, &force_callback);

	// Wait for connection
	if (!ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(ft_sensor_topic, ros::Duration(10.0))) {
		ROS_ERROR("timeout exceeded while waiting for message on topic %s",
		ft_sensor_topic.c_str());
		exit(EXIT_FAILURE);
	}

	ros::spin();

	return 0;
}