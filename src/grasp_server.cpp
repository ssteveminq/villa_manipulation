#include <string>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/PointIndices.h>
#include <pcl/cloud_iterator.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <boost/format.hpp>
#include <sensor_msgs/JointState.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <tmc_control_msgs/GripperApplyEffortGoal.h>
#include <tmc_manipulation_msgs/SafeJointChange.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>

#include <villa_manipulation/PickUpAction.h>
#include <villa_manipulation/PutDownAction.h>
#include <villa_manipulation/PerceptionPoseAction.h>
#include <villa_manipulation/ScanShelfAction.h>

#include <villa_manipulation/transform_utils.h>
#include <villa_manipulation/manipulator.h>
#include <villa_manipulation/sensor_capture.h>

#include <math.h>


class GraspActionServer {

protected:
  ros::NodeHandle nh_;

  villa_manipulation::Manipulator manipulator;
  ForceSensorCapture force_sensor_capture;

  geometry_msgs::Vector3 force_data;
  double resting_weight{};

  double OBJECT_LIFT_OFFSET = 0.04;
  double ARM_BASE_OFFSET = 0.30;

  double MAX_ARM_LIFT_JOINT = 0.69; // from hsr.io documentation
  double MAX_SHOULDER_HEIGHT = ARM_BASE_OFFSET + MAX_ARM_LIFT_JOINT;

  double ARM_LENGTH = 0.35; // distance from shoulder joint to wrist joints. measured on the robot

public:

  GraspActionServer() :
      manipulator() {

  }

  ~GraspActionServer() = default;

  void print_vector(const std::string &name, tf::Vector3 &vector) {
    ROS_INFO_STREAM(name << ": " << vector[0] << " " << vector[1] << " " << vector[2]);
  }

  double getCurrentWeight() {
    force_sensor_capture.getCurrentForce(force_data);
    return force_data.x;
  }

  void setDown(double shoulder_angle) {
    double current_weight = getCurrentWeight();
    double threshold = 0.2 * current_weight + 0.8 * resting_weight;
    double current_height;

    while (current_weight > threshold) {
      current_height = manipulator.get_trajectory_state(
          "/hsrb/arm_trajectory_controller/query_state").response.position[0];
      manipulator.move_arm_and_hand(current_height - 0.01, -M_PI / 2 + shoulder_angle, -shoulder_angle, 0.5, false);
      current_weight = getCurrentWeight();
    }
  }

  /**
  Initializes base readings of sensors and so on. Should be called when the robot is in a resting state.
  */
  bool initialize() {
    resting_weight = getCurrentWeight();
  }

  /** Takes in an CENTROID POSITION RELATIVE TO CAMERA of an object to pick up.
  Picks up the object by extending the arm laterally and moving so that the object is within the robot's gripper.
  After this method executes, the robot will be in the same location and orientation as it was before.
  Assumes the robot is facing in the same direction as the object needing to be picked up.
  @param point: The location of the object relative to the camera frame. */
  bool pickUp(geometry_msgs::Point object_location) {

    ROS_INFO("Calculating transform for arm");
    tf::Vector3 obj_head_rgbd_sensor_link;
    obj_head_rgbd_sensor_link[0] = object_location.x;
    obj_head_rgbd_sensor_link[1] = object_location.y;
    obj_head_rgbd_sensor_link[2] = object_location.z;
    print_vector("obj_head_rgbd_sensor_link", obj_head_rgbd_sensor_link);

    tf::Vector3 obj_arm_lift_link = transform_vec(obj_head_rgbd_sensor_link, "arm_lift_link", "head_rgbd_sensor_link");
    print_vector("arm_lift_link", obj_arm_lift_link);

    ROS_INFO("Moving arm horizontally and opening gripper");
    manipulator.move_arm(manipulator.get_joint_state("arm_lift_joint") + obj_arm_lift_link[2], -M_PI / 2);
    manipulator.open_gripper();

    ROS_INFO("Computing amount to move base");
    tf::Vector3 obj_hand_palm_link = transform_vec(obj_arm_lift_link, "hand_palm_link", "arm_lift_link");
    print_vector("obj_hand_palm_link", obj_hand_palm_link);

    ROS_INFO("Moving laterally");
    manipulator.slide_base(0.0, -obj_hand_palm_link[1]); // Horizontal
    manipulator.slide_base(obj_hand_palm_link[2] + 0.01, 0.0); // Forward/back TODO HACK

    ROS_INFO("Closing gripper");
    manipulator.grasp();

    double amountToRaiseObject = 0.04;
    manipulator.move_arm(manipulator.get_joint_state("arm_lift_joint") + OBJECT_LIFT_OFFSET, -M_PI / 2);

    ROS_INFO("Moving back");
    manipulator.slide_base(-obj_hand_palm_link[2], 0.0);
    manipulator.slide_base(0.0, obj_hand_palm_link[1]);

    manipulator.move_to_go();

    return manipulator.gripper_has_object();
  }

  /*
  Takes in a CENTROID POSITION RELATIVE TO BASE of where to put down something currently in the robot's hand.
  */
  bool putDown(geometry_msgs::Point place_location) {
    ROS_INFO("Calculating transform for arm");
    // tf::Vector3 obj_head_rgbd_sensor_link;
    // obj_head_rgbd_sensor_link[0] = place_location.x;
    // obj_head_rgbd_sensor_link[1] = place_location.y;
    // obj_head_rgbd_sensor_link[2] = place_location.z;
    // print_vector("obj_head_rgbd_sensor_link", obj_head_rgbd_sensor_link);

    tf::Vector3 obj_base_link;
    obj_base_link[0] = place_location.x;
    obj_base_link[1] = place_location.y;
    obj_base_link[2] = place_location.z;


    tf::Vector3 obj_arm_lift_link = transform_vec(obj_base_link, "arm_lift_link", "base_link");
    // tf::Vector3 obj_base_link = transform_vec(obj_head_rgbd_sensor_link, "base_link", "head_rgbd_sensor_link");
    // Stretch arm
    double shoulder_angle = 0;
    if (obj_base_link[2] < 0) {
      ROS_ERROR("Can't put down object because target location is under the ground.");
      return false;
    } else if (obj_base_link[2] <= MAX_SHOULDER_HEIGHT) {
      manipulator.move_arm(obj_base_link[2] - ARM_BASE_OFFSET, -M_PI / 2, false);
    } else if (obj_base_link[2] < MAX_SHOULDER_HEIGHT + ARM_LENGTH) {
      shoulder_angle = asin((obj_base_link[2] - MAX_SHOULDER_HEIGHT) / ARM_LENGTH);
      manipulator.move_arm_and_hand(MAX_ARM_LIFT_JOINT, -M_PI / 2 + shoulder_angle, -shoulder_angle);
    } else {
      ROS_ERROR("Can't put down object because target location is too high.");
      return false;
    }

    ROS_INFO("Computing amount to move base");
    // tf::Vector3 obj_hand_palm_link = transform_vec(obj_base_link, "hand_palm_link", "base_link");
    // print_vector("obj_hand_palm_link", obj_hand_palm_link);
    //
    // double horizontalDistance = -obj_hand_palm_link[1];
    // double forwardDistance = obj_hand_palm_link[2];

    tf::Vector3 hand_base_link = transform_vec(tf::Vector3(0, 0, 0), "base_link", "hand_palm_link");
    print_vector("hand_base_link", hand_base_link);

    double horizontalDistance = obj_base_link[1] - hand_base_link[1];
    double forwardDistance = obj_base_link[0] - hand_base_link[0];

    ROS_INFO("Moving laterally");
    manipulator.slide_base(0.0, horizontalDistance); // Horizontal
    manipulator.slide_base(forwardDistance, 0.0); // Forward/back

    ROS_INFO("Setting down");
    setDown(shoulder_angle);

    ROS_INFO("Opening gripper");
    manipulator.open_gripper();

    ROS_INFO("Moving back");
    manipulator.slide_base(-forwardDistance, 0.0);
    manipulator.slide_base(0.0, -horizontalDistance);

    return true;
  }

  bool scanShelf() {
    // prepare to scan
    //manipulator.turn_base(M_PI/2);
    //manipulator.move_arm(0.69,0.0);
    //manipulator.move_head(-M_PI/2,-M_PI/4);

    // scan
    double waitForArmDuration = 2.0;
    manipulator.move_arm(0.75, -M_PI, waitForArmDuration);

    // return to home
    //manipulator.move_head(0.0,0.0);
    //manipulator.turn_base(-M_PI/2);
    return true;
  }
};

GraspActionServer *executor_ = NULL;

void pickUp(const villa_manipulation::PickUpGoalConstPtr &goal,
            actionlib::SimpleActionServer<villa_manipulation::PickUpAction> *as) {
  ROS_INFO_STREAM("Picking up");
  bool succeeded = executor_->pickUp(goal->object_location);
  if (succeeded) {
    as->setSucceeded();
  } else {
    as->setAborted();
  }
}

void putDown(const villa_manipulation::PutDownGoalConstPtr &goal,
             actionlib::SimpleActionServer<villa_manipulation::PutDownAction> *as) {
  ROS_INFO_STREAM("Putting down");
  bool succeeded = executor_->putDown(goal->place_location);
  if (succeeded) {
    as->setSucceeded();
  } else {
    as->setAborted();
  }
}


void scanShelf(const villa_manipulation::ScanShelfGoalConstPtr &goal,
               actionlib::SimpleActionServer<villa_manipulation::ScanShelfAction> *as) {
  ROS_INFO_STREAM("Scanning shelf");
  bool succeeded = executor_->scanShelf();
  if (succeeded) {
    as->setSucceeded();
  } else {
    as->setAborted();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grasp_server");

  executor_ = new GraspActionServer();

  executor_->initialize();

  ros::NodeHandle n;

  actionlib::SimpleActionServer<villa_manipulation::PickUpAction> pickUpServer(n, "pickUp",
                                                                               boost::bind(&pickUp, _1, &pickUpServer),
                                                                               false);
  actionlib::SimpleActionServer<villa_manipulation::PutDownAction> putDownServer(n, "putDown", boost::bind(&putDown, _1,
                                                                                                           &putDownServer),
                                                                                 false);
  actionlib::SimpleActionServer<villa_manipulation::ScanShelfAction> scanShelfServer(n, "scanShelf",
                                                                                     boost::bind(&scanShelf, _1,
                                                                                                 &scanShelfServer),
                                                                                     false);
  pickUpServer.start();
  putDownServer.start();
  scanShelfServer.start();

  ROS_INFO("GRASP SERVERS ARE UP");
  ros::spin();
  return 0;
}
