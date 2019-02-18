// This node sends an empty goal to grasp_server

#include "plan_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char ** argv){
	ros::init(argc, argv, "test_executor");

	actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> ac("grasp",true);

	ROS_INFO("Waiting for server...");

	ac.waitForServer();

	ROS_INFO("Server has started");

	plan_execution::ExecutePlanGoal goal;
	ac.sendGoal(goal);

	return 0;
}
