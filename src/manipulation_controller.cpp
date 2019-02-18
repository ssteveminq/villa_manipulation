#include "manipulation_controller.h"

float Manipulator::get_joint_state(const std::string &name) {
    std::map<std::string, float>::iterator it = joint_states_.find(name);
    if (it != joint_states_.end()) {
        return it->second;
    } else {
        throw std::runtime_error("No such joint exists!");
    }
}

bool Manipulator::set_arm_joint_state(const std::string& joint_name, double joint_value, double duration) {
    if (joint_name.length() < 5) { // arbitrary number, does not check all cases
        ROS_ERROR("Controller::set_arm_joint_state(): invalid joint_name");
        return false;
    }

    wait_for_controller("arm_trajectory_controller");

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back(joint_name);
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(1);
    goal.trajectory.points[0].positions[0] = joint_value;
    goal.trajectory.points[0].velocities.resize(1);
    goal.trajectory.points[0].velocities[0] = 0.0;
    goal.trajectory.points[0].time_from_start = ros::Duration(duration);

    arm_client_.sendGoal(goal);
    wait_for_action(&arm_client_);

    return true;
}

bool Manipulator::set_arm_joint_state(const std::string& joint_name, double joint_value) {
    return set_arm_joint_state(joint_name, joint_value, 4.0);
}

bool Manipulator::rise_arm(double arm_height, double duration) {
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
    goal.trajectory.points[0].positions[1] = get_joint_state("arm_flex_joint");
    goal.trajectory.points[0].positions[2] = get_joint_state("arm_roll_joint");
    goal.trajectory.points[0].positions[3] = get_joint_state("wrist_flex_joint");
    goal.trajectory.points[0].positions[4] = get_joint_state("wrist_roll_joint");
    goal.trajectory.points[0].velocities.resize(5);

    for (size_t i = 0; i < 5; i++) {
        goal.trajectory.points[0].velocities[i] = 0.0;
    }



    goal.trajectory.points[0].time_from_start = ros::Duration(duration);
    arm_client_.sendGoal(goal);

    wait_for_action(&arm_client_);
}

bool Manipulator::rise_arm(double arm_height) {
    rise_arm(arm_height, 4.0);
}

bool Manipulator::move_arm(double arm_height, double arm_angle, double duration) {
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

    arm_client_.sendGoal(goal);

    wait_for_action(&arm_client_);
}

bool Manipulator::move_arm(double arm_height, double arm_angle) {
    move_arm(arm_height, arm_angle, 4.0);
}

bool Manipulator::grasp() {
  wait_for_controller("gripper_controller");
  tmc_control_msgs::GripperApplyEffortGoal goal;
  goal.effort = -0.01;
  ac.sendGoal(goal);
  bool finished_before_timeout = ac.waitForResult(ros::Duration(20.0));
  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state=  ac.getState();
    bool success = state == actionlib::SimpleClientGoalState::SUCCEEDED;
    if (!success) {ROS_ERROR("Grasp did not succeed");}
    return success;
  } else {
    ROS_ERROR("Grasp timeout");
    return false;
  }
}

bool Manipulator::move_arm_and_hand(double arm_height, double arm_angle, double hand_angle, double duration) {
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

    arm_client_.sendGoal(goal);

    wait_for_action(&arm_client_);
}

bool Manipulator::move_arm_and_hand(double arm_height, double arm_angle, double hand_angle) {
    move_arm_and_hand(arm_height, arm_angle, hand_angle, 4.0);
}

bool Manipulator::move_whole_arm(double arm_lift, double arm_flex, double arm_roll, double wrist_flex, double wrist_roll, double duration) {
    wait_for_controller("arm_trajectory_controller");

    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("arm_lift_joint");
    goal.trajectory.joint_names.push_back("arm_flex_joint");
    goal.trajectory.joint_names.push_back("arm_roll_joint");
    goal.trajectory.joint_names.push_back("wrist_flex_joint");
    goal.trajectory.joint_names.push_back("wrist_roll_joint");
    goal.trajectory.points.resize(1);

    goal.trajectory.points[0].positions.resize(5);
    goal.trajectory.points[0].positions[0] = arm_lift;
    goal.trajectory.points[0].positions[1] = arm_flex;
    goal.trajectory.points[0].positions[2] = arm_roll;
    goal.trajectory.points[0].positions[3] = wrist_flex;
    goal.trajectory.points[0].positions[4] = wrist_roll;
    goal.trajectory.points[0].velocities.resize(5);

    for (size_t i = 0; i < 5; i++) {
        goal.trajectory.points[0].velocities[i] = 0.0;
    }

    goal.trajectory.points[0].time_from_start = ros::Duration(duration);

    arm_client_.sendGoal(goal);

    wait_for_action(&arm_client_);
}

bool Manipulator::move_whole_arm(double arm_lift, double arm_flex, double arm_roll, double wrist_flex, double wrist_roll) {
    move_whole_arm(arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll, 4.0);
}

bool Manipulator::move_gripper(double amount) {
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

    gripper_client_.sendGoal(goal);

    wait_for_action(&gripper_client_);
}

bool Manipulator::move_gripper(bool isClosed) {
    if (isClosed) {
        move_gripper(-0.7);
    } else {
        move_gripper(1.1);
    }
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

    ROS_INFO_STREAM("roll " << roll << " " << "pitch " << pitch << " " << "yaw " << yaw);

    double current_turn = yaw;
    double x_offset = (cos(current_turn) * rel_coords[0]) - (sin(current_turn) * rel_coords[1]);
    double y_offset = (sin(current_turn) * rel_coords[0]) + (cos(current_turn) * rel_coords[1]);

    double new_x = location.x + x_offset;
    double new_y = location.y + y_offset;
    double new_t = current_turn + rel_coords[2];

    ROS_INFO_STREAM("Current location is " << location.x << " " << location.y << " " << current_turn);
    ROS_INFO_STREAM("Relative location is " << rel_coords[0] << " " << rel_coords[1] << " " << rel_coords[2]);
    ROS_INFO_STREAM("New location is " << new_x << " " << new_y << " " << new_t);

    return tf::Vector3(new_x, new_y, new_t);
}

bool Manipulator::move_base_relative(double x_relative_to_base, double y_relative_to_base, double turn_relative_to_base) {
    move_base_relative(x_relative_to_base, y_relative_to_base, turn_relative_to_base, 4.0);
}

bool Manipulator::move_base_relative(double x_relative_to_base, double y_relative_to_base, double turn_relative_to_base, double duration) {
    tf::Vector3 goal_coords = compute_new_location(tf::Vector3(x_relative_to_base, y_relative_to_base, turn_relative_to_base));

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
    goal.trajectory.points[0].velocities.resize(3);
    for (size_t i = 0; i < 3; i++) {
        goal.trajectory.points[0].velocities[i] = 0.0;
    }
    goal.trajectory.points[0].time_from_start = ros::Duration(duration);

    base_client_.sendGoal(goal);

    wait_for_action(&base_client_);
}

bool Manipulator::turn_base(double turn, double duration) {
    move_base_relative(0.0, 0.0, turn, duration);
}

bool Manipulator::turn_base(double turn) {
    move_base_relative(0.0, 0.0, turn);
}

bool Manipulator::slide_base(double x, double y, double duration) {
    move_base_relative(x, y, 0.0, duration);
}

bool Manipulator::slide_base(double x, double y) {
    move_base_relative(x, y, 0.0);
}

bool Manipulator::move_head(double pan_angle, double tilt_angle) {
    wait_for_controller("head_trajectory_controller");

    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("head_pan_joint");
    goal.trajectory.joint_names.push_back("head_tilt_joint");
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = pan_angle;
    goal.trajectory.points[0].positions[1] = tilt_angle;
    goal.trajectory.points[0].velocities.resize(2);
    for(size_t i = 0; i < 2; i++) {
        goal.trajectory.points[0].velocities[i] = 0.0;
    }

    goal.trajectory.points[0].time_from_start = ros::Duration(2.5);

    head_client_.sendGoal(goal);

    wait_for_action(&head_client_);
}
