#include <villa_manipulation/manipulator.h>
#include <ros/duration.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <tmc_planning_msgs/PlanWithJointGoals.h>
#include <tmc_manipulation_msgs/BaseMovementType.h>
#include <tf/tf.h>

using namespace std;

namespace villa_manipulation {
float Manipulator::get_joint_state(const std::string &name) {
  auto as_map = as_joint_map(joint_states_);
  auto it = as_map.find(name);
  if (it != as_map.end()) {
    return it->second;
  } else {
    throw std::runtime_error("No such joint exists!");
  }
}


trajectory_msgs::JointTrajectory extract(const trajectory_msgs::JointTrajectory &trajectory, const std::vector<std::string> &joint_names, const sensor_msgs::JointState &joint_state) {
  const int num_points = trajectory.points.size();
  const int num_joints = joint_names.size();
  const int num_source_joints = trajectory.joint_names.size();
  std::vector<int> index_map(num_joints, -1);
  // Figure out the map from index into joint names to index into trajectory's joint names
  for (int i = 0; i < joint_names.size(); i++) {
    for (int j = 0; j < trajectory.joint_names.size(); j++) {
      if (joint_names[i] == trajectory.joint_names[j]) {
        index_map[i] = j;
      }
    }
  }

  trajectory_msgs::JointTrajectory result;
  result.joint_names = joint_names;
  result.points = vector<trajectory_msgs::JointTrajectoryPoint>(num_points);

  for (int i = 0; i < num_points; ++i) {
    auto &target = result.points[i];
    auto &source = trajectory.points[i];
    target.positions = vector<double>(num_joints, 0);
    target.velocities = vector<double>(num_joints, 0);
    target.accelerations = vector<double>(num_joints, 0);
    target.effort = vector<double>(num_joints, 0);
    target.time_from_start = source.time_from_start;
    // Check the point has enough elements
    // FIXME: If the given trajectory is well-formed, this check is not
    //        necessary. Actually we meet malformed trajectory sometime.
    bool has_velocities = source.velocities.size() == num_source_joints;
    bool has_efforts = source.effort.size() == num_source_joints;
    bool has_accelerations = source.accelerations.size() == num_source_joints;
    for (int j = 0; j < num_joints; ++j) {
      if (index_map[j] != -1) {
        auto pos = source.positions[index_map[j]];
        target.positions[j] = pos;
        if (has_velocities) {
          auto vel = source.velocities[index_map[j]];
          target.velocities[j] = vel;
        }
        if (has_accelerations) {
          auto acc = source.accelerations[index_map[j]];
          target.accelerations[j] = acc;
        }
        if (has_efforts) {
          auto eff = source.effort[index_map[j]];
          target.effort[j] = eff;
        }
      } else {
        auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_names[j]);
        if (it == joint_state.name.end()) {
          ROS_ERROR_STREAM("joint state did not publish " << joint_names[j]);
        }
        long index = std::distance(joint_state.name.begin(), it);
        auto angle = joint_state.position[index];
        target.positions[j] = angle;
        target.velocities[j] = 0.0;
        target.accelerations[j] = 0.0;
        target.effort[j] = 0.0;
      }
    }
  }

  return result;
}

bool Manipulator::move_arm(double arm_height, double arm_angle, double duration, bool blocking) {
  wait_for_controller("arm_trajectory_controller");

  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back("arm_lift_joint");
  goal.trajectory.joint_names.push_back("arm_flex_joint");
  goal.trajectory.joint_names.push_back("arm_roll_joint");
  goal.trajectory.joint_names.push_back("wrist_flex_joint");
  goal.trajectory.joint_names.push_back("wrist_roll_joint");
  goal.trajectory.points.resize(1);

  goal.trajectory.points[0].positions.resize(5);
  goal.trajectory.points[0].positions[0] = arm_height;
  goal.trajectory.points[0].positions[1] = arm_angle;
  goal.trajectory.points[0].positions[2] = 0;
  goal.trajectory.points[0].positions[3] = 0;
  goal.trajectory.points[0].positions[4] = 0;
  goal.trajectory.points[0].velocities.resize(5);

  for (size_t i = 0; i < 5; i++) {
    goal.trajectory.points[0].velocities[i] = 0.0;
  }

  goal.trajectory.points[0].time_from_start = ros::Duration(duration);

  arm_controller.client.sendGoal(goal);
  if (blocking) {
    return wait_for_action(arm_controller.client);
  }
  return true;
}

bool Manipulator::move_arm(double arm_height, double arm_angle, bool blocking) {
  return move_arm(arm_height, arm_angle, 4.0, blocking);
}

bool
Manipulator::move_arm_and_hand(double arm_height, double arm_angle, double hand_angle, double duration, bool blocking) {
  wait_for_controller("arm_trajectory_controller");

  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back("arm_lift_joint");
  goal.trajectory.joint_names.push_back("arm_flex_joint");
  goal.trajectory.joint_names.push_back("arm_roll_joint");
  goal.trajectory.joint_names.push_back("wrist_flex_joint");
  goal.trajectory.joint_names.push_back("wrist_roll_joint");
  goal.trajectory.points.resize(1);

  goal.trajectory.points[0].positions.resize(5);
  goal.trajectory.points[0].positions[0] = arm_height;
  goal.trajectory.points[0].positions[1] = arm_angle;
  goal.trajectory.points[0].positions[2] = 0;
  goal.trajectory.points[0].positions[3] = hand_angle;
  goal.trajectory.points[0].positions[4] = 0;
  goal.trajectory.points[0].velocities.resize(5);

  for (size_t i = 0; i < 5; i++) {
    goal.trajectory.points[0].velocities[i] = 0.0;
  }

  goal.trajectory.points[0].time_from_start = ros::Duration(duration);

  arm_controller.client.sendGoal(goal);
  if (blocking) {
    return wait_for_action(arm_controller.client);
  }
  return true;
}

bool Manipulator::move_arm_and_hand(double arm_height, double arm_angle, double hand_angle, bool blocking) {
  move_arm_and_hand(arm_height, arm_angle, hand_angle, 4.0, blocking);
}

bool Manipulator::grasp(double effort) {
  wait_for_controller("gripper_controller");
  tmc_control_msgs::GripperApplyEffortGoal goal;
  goal.effort = effort;
  gripper_effort_client.sendGoal(goal);
  bool finished_before_timeout = gripper_effort_client.waitForResult(ros::Duration(20.0));
  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = gripper_effort_client.getState();
    bool success = state == actionlib::SimpleClientGoalState::SUCCEEDED;
    if (!success) { ROS_ERROR("Grasp did not succeed"); }
    return success;
  } else {
    ROS_ERROR("Grasp timeout");
    return false;
  }
}

bool Manipulator::move_gripper(double amount, bool blocking) {
  wait_for_controller("gripper_controller");

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back("hand_motor_joint");

  goal.trajectory.points.resize(1);

  goal.trajectory.points[0].positions.resize(1);
  goal.trajectory.points[0].positions[0] = amount;
  goal.trajectory.points[0].velocities.resize(1);
  goal.trajectory.points[0].velocities[0] = 0.0;
  goal.trajectory.points[0].effort.resize(1);
  goal.trajectory.points[0].effort[0] = 0.1;
  goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

  gripper_controller.client.sendGoal(goal);
  if (blocking) {
    return wait_for_action(gripper_controller.client);
  }
  return true;
}

bool Manipulator::close_gripper() {
  return move_gripper(-0.3, false);
}

bool Manipulator::open_gripper() {
  return move_gripper(1.4, false);
}

tf::Vector3 Manipulator::compute_new_location(tf::Vector3 rel_coords) {
  geometry_msgs::Point location;
  geometry_msgs::Quaternion orientation;
  odom_.getCurrentOdometry(location, orientation);


  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation, quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  //ROS_INFO_STREAM("roll " << roll << " " << "pitch " << pitch << " " << "yaw " << yaw);

  double current_turn = yaw;
  double x_offset = (cos(current_turn) * rel_coords[0]) - (sin(current_turn) * rel_coords[1]);
  double y_offset = (sin(current_turn) * rel_coords[0]) + (cos(current_turn) * rel_coords[1]);

  double new_x = location.x + x_offset;
  double new_y = location.y + y_offset;
  double new_t = current_turn + rel_coords[2];

  //ROS_INFO_STREAM("Current location is " << location.x << " " << location.y << " " << current_turn);
  //ROS_INFO_STREAM("Relative location is " << rel_coords[0] << " " << rel_coords[1] << " " << rel_coords[2]);
  //ROS_INFO_STREAM("New location is " << new_x << " " << new_y << " " << new_t);

  return {new_x, new_y, new_t};
}

bool
Manipulator::move_base_relative(double x, double y, double theta,
                                bool blocking) {
  tf::Vector3 goal_coords = compute_new_location(
      tf::Vector3(x, y, theta));

  wait_for_controller("omni_base_controller");

  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back("odom_x");
  goal.trajectory.joint_names.push_back("odom_y");
  goal.trajectory.joint_names.push_back("odom_t");

  goal.trajectory.points.resize(1);

  goal.trajectory.points[0].positions.resize(3);
  goal.trajectory.points[0].positions[0] = goal_coords[0];
  goal.trajectory.points[0].positions[1] = goal_coords[1];
  goal.trajectory.points[0].positions[2] = goal_coords[2];
  goal.trajectory.points[0].velocities = vector<double>(3, 0.0);
  goal.trajectory.points[0].time_from_start = ros::Duration(4.0);

  base_controller.client.sendGoal(goal);
  if (blocking) {
    return wait_for_action(base_controller.client);
  }
  return true;
}

bool Manipulator::turn_base(double turn, bool blocking) {
  return move_base_relative(0.0, 0.0, turn, blocking);
}

bool Manipulator::slide_base(double x, double y, bool blocking) {
  return move_base_relative(x, y, 0.0, blocking);
}

bool Manipulator::move_head(double pan_angle, double tilt_angle, bool blocking) {
  wait_for_controller("head_trajectory_controller");

  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back("head_pan_joint");
  goal.trajectory.joint_names.push_back("head_tilt_joint");
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(2);
  goal.trajectory.points[0].positions[0] = pan_angle;
  goal.trajectory.points[0].positions[1] = tilt_angle;
  goal.trajectory.points[0].velocities.resize(2);
  for (size_t i = 0; i < 2; i++) {
    goal.trajectory.points[0].velocities[i] = 0.0;
  }

  goal.trajectory.points[0].time_from_start = ros::Duration(2.5);

  head_controller.client.sendGoal(goal);
  if (blocking) {
    return wait_for_action(head_controller.client);
  }
  return true;
}

bool Manipulator::move_to_joint_positions_unsafe(const JointStateMap &goal_map, bool blocking) {
  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.points.resize(1);
  for (const auto &pair: goal_map) {
    goal.trajectory.joint_names.push_back(pair.first);
    goal.trajectory.points[0].positions.push_back(pair.second);
    goal.trajectory.points[0].velocities.push_back(0.0);
  }

  goal.trajectory.points[0].time_from_start = ros::Duration(2.5);

  arm_controller.client.sendGoal(goal);
  if (blocking) {
    return wait_for_action(arm_controller.client);
  }
  return true;
}

bool Manipulator::move_to_neutral(bool blocking) {
  map<string, float> goal = {
      {"arm_lift_joint",   0.0},
      {"arm_flex_joint",   0.0},
      {"arm_roll_joint",   0.0},
      {"wrist_flex_joint", -1.57},
      {"wrist_roll_joint", 0.0}
  };
  
  return move_to_joint_positions(goal, blocking);
}

/**
 * @brief Move joints to a suitable pose for moving a mobile base.
 */
bool Manipulator::move_to_go(bool blocking) {
  map<string, float> goal = {
      {"arm_flex_joint",   0.0},
      {"arm_lift_joint",   0.0},
      {"arm_roll_joint",   -1.57},
      {"wrist_flex_joint", -1.57},
      {"wrist_roll_joint", 0.0},
      {"head_pan_joint",   0.0},
      {"head_tilt_joint",  0.0}
  };
  return move_to_joint_positions(goal, blocking);
}

bool Manipulator::move_to_joint_positions(const JointStateMap &target, bool blocking) {
  // Usage for the TMC planning services is given in
  // /opt/ros/kinetic/lib/python2.7/dist-packages/hsrb_interface/joint_group.py

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.waitForTransform("odom", "base_footprint", ros::Time(0), ros::Duration(3));
    listener.lookupTransform("odom", "base_footprint", ros::Time(0), transform);

  } catch (tf::TransformException &ex) {
  }

  auto planning_request = tmc_planning_msgs::PlanWithJointGoalsRequest();
  tf::pointTFToMsg(transform.getOrigin(), planning_request.origin_to_basejoint.position);
  tf::quaternionTFToMsg(transform.getRotation(), planning_request.origin_to_basejoint.orientation);
  wait_for_data(3);
  planning_request.initial_joint_state = *joint_states_;

  auto target_state = as_joint_state(target);
  planning_request.use_joints = target_state.name;
  tmc_planning_msgs::JointPosition goal_position;
  goal_position.position = target_state.position;
  planning_request.goal_joint_states.push_back(goal_position);

  planning_request.max_iteration = max_planning_iterations;
  planning_request.timeout = ros::Duration(max_planning_time);
  planning_request.base_movement_type.val = tmc_manipulation_msgs::BaseMovementType::PLANAR;
  planning_request.weighted_joints = {"_linear_base", "_rotational_base"};
  planning_request.weight = {linear_weight, angular_weight};

  tmc_planning_msgs::PlanWithJointGoalsResponse response;
  if (!joint_planning_client.waitForExistence(ros::Duration(3))) {
    return false;
  }
  bool succeeded = joint_planning_client.call(planning_request, response);
  if (!succeeded) {
    return false;
  }
  if (response.error_code.val != tmc_manipulation_msgs::ArmManipulationErrorCodes::SUCCESS) {
    return false;
  }
  response.base_solution.header.frame_id = "odom";
  auto constrained = constrain_trajectories(response.solution);
  if (!constrained) {
    return false;
  }
  return execute_trajectory(*constrained, blocking);

}

boost::optional<trajectory_msgs::JointTrajectory> Manipulator::constrain_trajectories(const trajectory_msgs::JointTrajectory &trajectory) {
  auto start_state = *joint_states_;
  tmc_manipulation_msgs::FilterJointTrajectoryRequest request;
  tmc_manipulation_msgs::FilterJointTrajectoryResponse response;
  request.trajectory = trajectory;
  auto whole_name = trajectory.joint_names;
  whole_name.emplace_back("base_roll_joint");
  vector<double> whole_pos;
  const auto &names = start_state.name;
  for (const auto& name: whole_name) {
    auto it = std::find(names.begin(), names.end(), name);
    auto index = distance(names.begin(), it);
    whole_pos.emplace_back(start_state.position[index]);
  }
  request.start_state.joint_state.name = whole_name;
  request.start_state.joint_state.position = whole_pos;
  bool success = whole_body_timeopt_filter_client.call(request, response);

  if (!success || response.error_code.val != tmc_manipulation_msgs::ArmNavigationErrorCodes::SUCCESS) {
    return {};
  }
  return response.trajectory;

}

bool Manipulator::execute_trajectory(const trajectory_msgs::JointTrajectory &trajectory, bool blocking) {
  set<reference_wrapper<TrajectoryController>> clients;
  for (const auto &controller: position_controllers) {
    for (const auto &joint: trajectory.joint_names) {
      const auto &names = controller.get().joint_names;
      if (std::find(names.begin(), names.end(), joint) != names.end()) {
        clients.insert(controller);
      }
    }
  }
  assert(clients.size() > 0);
  for (const auto &controller: clients) {
    auto extracted_traj = extract(trajectory, controller.get().joint_names, *joint_states_);
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = extracted_traj;
    controller.get().client.sendGoal(goal);
  }
  if (blocking) {
    return wait_for_controllers(clients);
  }
  return true;
}

bool Manipulator::wait_for_controllers(const set<std::reference_wrapper<TrajectoryController>> &controllers) {
  ros::Rate watch_rate(30.);
  set<int> ok_set = {actionlib_msgs::GoalStatus::PENDING, actionlib_msgs::GoalStatus::ACTIVE, actionlib_msgs::GoalStatus::SUCCEEDED};
  while (ros::ok()) {
    vector<int> states;
    vector<bool> ok;
    for (const auto &controller: controllers) {
      auto state = controller.get().client.getState().state_;
      states.emplace_back(state);
      ok.emplace_back(ok_set.count(state) != 0);
    }
    bool all_succeeded = true;
    for (int i = 0; i < ok.size(); i++) {
      bool okayness = ok.at(i);
      if (!okayness) {
        // TODO: Write a log message about which controller failed
        return false;
      }
      all_succeeded &= states.at(i) == actionlib_msgs::GoalStatus::SUCCEEDED;
    }
    if (all_succeeded) {
      return true;
    }
    watch_rate.sleep();
  }

}


bool Manipulator::gripper_has_object() { //TODO Maybe different for different robot
  wait_for_data(10);
  auto joint_map = as_joint_map(joint_states_);
  return joint_map["hand_l_spring_proximal_joint"]
         + joint_map["hand_l_spring_proximal_joint"] > 1;
}

bool Manipulator::wait_for_data(double timeout) {
  heard_joint_states = false;

  ros::Rate r(40);

  // Negative timeout is interpreted as no timeout
  bool use_timeout = timeout > 0.0;

  ros::Time end = ros::Time::now() + ros::Duration(timeout);
  while (ros::ok()) {
    ros::spinOnce();

    if (heard_joint_states)
      return true;

    r.sleep();

    if (use_timeout && ros::Time::now() > end) {
      return false;
    }
  }
}

Manipulator::Manipulator(bool in_simulation) :
    nh_(),
    arm_controller("/hsrb/arm_trajectory_controller"),
    base_controller("/hsrb/omni_base_controller"),
    gripper_controller("/hsrb/gripper_controller"),
    head_controller("/hsrb/head_trajectory_controller"),
    heard_joint_states(false),
    gripper_effort_client("/hsrb/gripper_controller/grasp", true),
    in_simulation(in_simulation) {

  joint_states_sub_ = nh_.subscribe("/hsrb/robot_state/joint_states", 1, &Manipulator::updateJointStates,
                                    this);

  list_controllers_client = nh_.serviceClient<controller_manager_msgs::ListControllers>(
      "/hsrb/controller_manager/list_controllers");

  joint_planning_client = nh_.serviceClient<tmc_planning_msgs::PlanWithJointGoals>("/plan_with_joint_goals");

  whole_body_timeopt_filter_client = nh_.serviceClient<tmc_manipulation_msgs::FilterJointTrajectory>("/filter_hsrb_trajectory");

  position_controllers.push_back(std::ref(arm_controller));
  position_controllers.push_back(std::ref(base_controller));
  position_controllers.push_back(std::ref(gripper_controller));
  position_controllers.push_back(std::ref(head_controller));


  if (ros::param::has("/simulation")) {
    in_simulation = ros::param::param("/simulation", false);
  }

  ROS_INFO_STREAM("Waiting for controller manager...");
  list_controllers_client.waitForExistence();
}

}