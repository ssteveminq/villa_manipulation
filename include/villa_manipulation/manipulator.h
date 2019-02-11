#pragma once

#include <string>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include "sensor_capture.h"
#include "transform_utils.h"
#include <tf/transform_listener.h>

#include <tmc_planning_msgs/PlanWithJointGoals.h>
#include <tmc_manipulation_msgs/FilterJointTrajectory.h>


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
struct TrajectoryController {
  const std::string name;
  Client client;
  std::vector<std::string> joint_names;

  explicit TrajectoryController(const std::string &name): name(name), client(name + "/follow_joint_trajectory", true) {
    auto param_name = name + "/joints";
    assert(ros::param::has(param_name));
    ros::param::get(param_name, joint_names);
  }
  inline bool operator < (const TrajectoryController &rhs) const {
    return name < rhs.name;
  }
};

inline bool operator < (const std::reference_wrapper<TrajectoryController> &lhs, const std::reference_wrapper<TrajectoryController> &rhs) {
  return lhs.get() < rhs.get();
}


typedef std::map<std::string, float> JointStateMap;
namespace villa_manipulation {
class Manipulator {
  // Instance tunable parameters
  float action_timeout = 10;
  float max_planning_time = 10;
  float planning_goal_generation = 0.3f;
  float planning_goal_deviation = 0.3f;
  uint max_planning_iterations = 100000;

  float linear_weight = 3.0;
  float angular_weight = 1.0;
private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_states_sub_;
  ros::ServiceClient list_controllers_client;
  TrajectoryController arm_controller;
  TrajectoryController base_controller;
  TrajectoryController gripper_controller;
  TrajectoryController head_controller;
  std::vector<std::reference_wrapper<TrajectoryController>> position_controllers;
  ros::ServiceClient joint_planning_client;
  ros::ServiceClient whole_body_timeopt_filter_client;
  OdometryCapture odom_;
  sensor_msgs::JointState::ConstPtr joint_states_;
  actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction> gripper_effort_client;

  bool in_simulation;

  void wait_for_controller(const std::string &name) {
    if (in_simulation && name == "gripper_controller") {
      return;
    }
    controller_manager_msgs::ListControllers list_controllers;
    bool running = false;
    while (!running && ros::ok()) {
      ros::Duration(0.1).sleep();
      if (list_controllers_client.call(list_controllers)) {
        for (auto c : list_controllers.response.controller) {
          if (c.name == name && c.state == "running") {
            running = true;
          }
        }
      }
    }
  }

  bool wait_for_action(Client &client) {
    bool finished_before_timeout = client.waitForResult(ros::Duration(action_timeout));
    if (finished_before_timeout) {
      actionlib::SimpleClientGoalState state = client.getState();
      //ROS_INFO("Action finished: %s", state.toString().c_str());
      return true;
    }

    ROS_ERROR("Action did not finish before timeout");
    return false;
  }

  tf::Vector3 compute_new_location(tf::Vector3 rel_coords);

public:


  explicit Manipulator(bool in_simulation = false);

  ~Manipulator() {
    std::cout << "Destroyed manipulation" << std::endl;
  }

  control_msgs::QueryTrajectoryState get_trajectory_state(const std::string &name) {
    ros::ServiceClient query_client = nh_.serviceClient<control_msgs::QueryTrajectoryState>(name);
    control_msgs::QueryTrajectoryState query;
    query.request.time = ros::Time::now();

    if (!query_client.call(query)) {
      ROS_ERROR("Graspserver: failed to receieve state for %s", name.c_str());
    }

    return query;
  }

  float get_joint_state(const std::string &name);

  bool move_arm(double arm_height, double arm_angle, double duration, bool blocking = true);

  bool move_arm(double arm_height, double arm_angle, bool blocking = true);

  bool move_arm_and_hand(double arm_height, double arm_angle, double hand_angle, double duration, bool blocking = true);

  bool move_arm_and_hand(double arm_height, double arm_angle, double hand_angle, bool blocking = true);

  bool move_base_relative(double x, double y, double theta,
                          bool blocking = true);

  bool grasp(double effort = -0.01);

  bool move_gripper(double amount, bool blocking = true);

  bool close_gripper();

  bool open_gripper();

  bool slide_base(double x, double y, bool blocking = true);

  bool turn_base(double turn_amount, bool blocking = true);

  bool move_head(double pan_angle, double tilt_angle, bool blocking = true);

  bool move_to_joint_positions_unsafe(const JointStateMap &goal, bool blocking = true);

  bool move_to_joint_positions(const JointStateMap &target, bool blocking = true);

  bool gripper_has_object();

  void updateJointStates(const sensor_msgs::JointState::ConstPtr &joint_states) {
    heard_joint_states = true;
    this->joint_states_ = joint_states;
  };

  JointStateMap as_joint_map(const sensor_msgs::JointState::ConstPtr &joint_states) {
    JointStateMap jsm;
    for (unsigned int i = 0; i < joint_states->name.size(); i++) {
      std::string joint = joint_states->name[i];
      if (jsm.find(joint) != jsm.end()) jsm[joint] = joint_states->position[i];
      else jsm.insert(std::pair<std::string, float>(joint, joint_states->position[i]));
    }
    return jsm;
  };

  sensor_msgs::JointState as_joint_state(const JointStateMap &jsm) {
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    for (const auto &entry: jsm) {
      joint_state.name.push_back(entry.first);
      joint_state.position.push_back(entry.second);
      joint_state.effort.push_back(0.0);
      joint_state.velocity.push_back(0.0);
    }
    return joint_state;
  }

  bool execute_trajectory(const trajectory_msgs::JointTrajectory &trajectory, bool blocking = true);

  boost::optional<trajectory_msgs::JointTrajectory> constrain_trajectories(const trajectory_msgs::JointTrajectory &trajectory);

  bool wait_for_controllers(const std::set<std::reference_wrapper<TrajectoryController>> &controllers);

  bool move_to_go(bool blocking = true);

  bool move_to_neutral(bool blocking = true);

  bool wait_for_data(double timeout);

  bool heard_joint_states{};
};
}
