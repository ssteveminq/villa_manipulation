#include <string>
#include <cmath>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/PointIndices.h>
#include <pcl/cloud_iterator.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/String.h>
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
#include <villa_manipulation/PickUpCutleryAction.h>
#include <villa_manipulation/PickUpPlateAction.h>
#include <villa_manipulation/PutDownAction.h>
#include <villa_manipulation/PutDownDishwasherAction.h>
#include <villa_manipulation/PerceptionPoseAction.h>
#include <villa_manipulation/ScanShelfAction.h>

//#include "plan_execution/ExecutePlanAction.h"
//#include "bwi_perception/PerceiveTabletopScene.h"
#include "villa_manipulation/transform_utils.h"
#include "manipulation_controller.h"
#include "sensor_capture.h"
#include "AlignRobotToSurface.h"

#define PI 3.1415

class GraspAction {

protected:
    ros::NodeHandle nh_;

    Manipulator manipulator;
    AlignRobotToSurface robot_aligner;
    ForceSensorCapture force_sensor_capture;

    geometry_msgs::Vector3 force_data;
    double resting_weight;

    double OBJECT_LIFT_OFFSET = 0.04;
    double ARM_BASE_OFFSET = 0.30;
    double MAX_ARM_LIFT_JOINT = 0.69; // from hsr.io documentation
    double MAX_SHOULDER_HEIGHT = ARM_BASE_OFFSET + MAX_ARM_LIFT_JOINT;

    double ARM_LENGTH = 0.35; // distance from shoulder joint to wrist joints. measured on the robot

    double toPlusMinusPiHalvesRange(double angle){
        return fmod(angle + PI/2, PI) - PI/2;
    }
public:

    GraspAction(void) :
    manipulator(nh_), robot_aligner(nh_) {}

    ~GraspAction(void){}

    void print_vector(std::string name, tf::Vector3& vector) {
        ROS_INFO_STREAM(name << ": " << vector[0] << " " << vector[1] << " " << vector[2]);
    }

    double getCurrentWeight() {
        force_sensor_capture.getCurrentForce(force_data);
        return force_data.x;
    }

    // TODO This function is deprecated and its overloaded version with no parameters should be used instead. I don't delete it just because Robocup is in one week and I don't want to change things that work and mess up
    void setDown(double shoulder_angle) {
        double current_weight = abs(getCurrentWeight());
        double threshold = 0.2 * current_weight + 0.8 * resting_weight;
        //double threshold = 0.2 * current_weight + 0.8 * resting_weight;
        double current_height;

        ROS_INFO("SET DOWN");
        std::cout << "Current weight: " << current_weight << std::endl;
        std::cout << "Resting weight: " << resting_weight << std::endl;
        std::cout << "Threshold:      " << threshold << std::endl;

        while (current_weight > threshold) {
            current_height = manipulator.get_trajectory_state("/hsrb/arm_trajectory_controller/query_state").response.position[0];
            std::cout << "Current weight: " << current_weight << std::endl;
            manipulator.move_arm_and_hand(current_height - 0.015, -PI/2 + shoulder_angle, -shoulder_angle, 0.5);
            current_weight = abs(getCurrentWeight());
        }
        ros::Duration(1.5).sleep();
    }

    void setDown() {
        double current_weight = abs(getCurrentWeight());
        //double threshold = 0.4 * current_weight + 0.6 * resting_weight;
        double threshold = 1.2*current_weight;
        double current_height;

        std::cout << "Current weight: " << current_weight << std::endl;
        std::cout << "Resting weight: " << resting_weight << std::endl;
        std::cout << "Threshold:      " << threshold << std::endl;


        while (current_weight < threshold) {
            current_height = manipulator.get_trajectory_state("/hsrb/arm_trajectory_controller/query_state").response.position[0];

            std::cout << "Current weight: " << current_weight << std::endl;
            manipulator.rise_arm(current_height - 0.01, 1.0);
            current_weight = abs(getCurrentWeight());
        }
        ros::Duration(1.5).sleep();
    }

    void slightlyRiseUpArm(double shoulder_angle){
      double current_height = manipulator.get_trajectory_state("/hsrb/arm_trajectory_controller/query_state").response.position[0];
      manipulator.move_arm_and_hand(current_height + 0.02, -PI/2 + shoulder_angle, -shoulder_angle, 0.5);
    }

    /**
    Initializes base readings of sensors and so on. Should be called when the robot is in a resting state.
    */
    bool initialize() {
        resting_weight = abs(getCurrentWeight());
    }

    /**
    Moves the robot to a pose where it can easily see the tabletop.
    */
    bool moveToPerceptionPose() {
        ROS_INFO("Moving head to correct angle");
        manipulator.move_head(0, -0.55);

        double waitForArmDuration = 3.0;
        ROS_INFO("Moving arm down");
        manipulator.move_arm(0.75, -PI, waitForArmDuration);

       ros::Duration(waitForArmDuration).sleep();
        return true;
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

        ROS_INFO("Placing arm vertically and opening gripper");
        manipulator.move_arm(manipulator.get_joint_state("arm_lift_joint") + obj_arm_lift_link[2], -PI / 2);
        manipulator.move_gripper(false);

        ROS_INFO("Computing amount to move base");
        tf::Vector3 obj_hand_palm_link = transform_vec(obj_arm_lift_link, "hand_palm_link", "arm_lift_link");
        print_vector("obj_hand_palm_link", obj_hand_palm_link);

        ROS_INFO("Moving laterally");
        manipulator.slide_base(0.0, -obj_hand_palm_link[1]); // Horizontal
        manipulator.slide_base(obj_hand_palm_link[2] + 0.01, 0.0); // Forward/back TODO HACK

        ROS_INFO("Closing gripper");
        manipulator.grasp();

        manipulator.move_arm(manipulator.get_joint_state("arm_lift_joint") + OBJECT_LIFT_OFFSET, -PI/2);

        ROS_INFO("Moving back");
        manipulator.slide_base(-obj_hand_palm_link[2], 0.0);
        manipulator.slide_base(0.0, obj_hand_palm_link[1]);

        manipulator.move_whole_arm(0.0, 0.0, 0.0, -PI/2, 0.0);

        if (manipulator.gripper_has_object()){
            return true;
        } else {
            return false;
        }
    }

    /** Takes in an CENTROID POSITION RELATIVE TO CAMERA of an object to pick up.
    Picks up the object by extending the arm laterally and moving so that the object is within the robot's gripper.
    After this method executes, the robot will be in the same location and orientation as it was before.
    Assumes the robot is facing in the same direction as the object needing to be picked up.
    @param point: The location of the object relative to the camera frame. */
    bool pickUpCutlery(geometry_msgs::Point object_location, geometry_msgs::Vector3 orientation) {

        ROS_INFO("Calculating transform for arm");
        tf::Vector3 obj_head_rgbd_sensor_link;
        obj_head_rgbd_sensor_link[0] = object_location.x;
        obj_head_rgbd_sensor_link[1] = object_location.y;
        obj_head_rgbd_sensor_link[2] = object_location.z;
        print_vector("obj_head_rgbd_sensor_link", obj_head_rgbd_sensor_link);

        tf::Vector3 obj_arm_lift_link = transform_vec(obj_head_rgbd_sensor_link, "arm_lift_link", "head_rgbd_sensor_link");
        print_vector("arm_lift_link", obj_arm_lift_link);

        tf::Vector3 orientation_head_rgbd_sensor_link;
        orientation_head_rgbd_sensor_link[0] = orientation.x;
        orientation_head_rgbd_sensor_link[1] = orientation.y;
        orientation_head_rgbd_sensor_link[2] = orientation.z;
        tf::Vector3 orientation_base_link = transform_vec(orientation_head_rgbd_sensor_link, "base_link", "head_rgbd_sensor_link");
        double wrist_roll = toPlusMinusPiHalvesRange( - asin(orientation_base_link[1] / orientation_base_link[0]));

        ROS_INFO("Moving arm vertically (20cm above object) and opening gripper");
        manipulator.move_whole_arm(manipulator.get_joint_state("arm_lift_joint") + obj_arm_lift_link[2] + 0.2, -PI / 2, 0.0, -PI/2, wrist_roll);
        manipulator.move_gripper(false);

        ROS_INFO("Computing amount to move base");
        tf::Vector3 obj_arm_roll_link = transform_vec(obj_arm_lift_link, "arm_roll_link", "arm_lift_link");
        print_vector("obj_arm_roll_link", obj_arm_roll_link);

        ROS_INFO("Moving laterally");
        manipulator.slide_base(0.0, obj_arm_roll_link[1]); // Horizontal
        manipulator.slide_base(obj_arm_roll_link[2], 0.0); // Forward/back TODO HACK

        ROS_INFO("Setting down");
        setDown();

        ROS_INFO("Closing gripper");
        manipulator.grasp();

        manipulator.move_whole_arm(manipulator.get_joint_state("arm_lift_joint") + OBJECT_LIFT_OFFSET, -PI/2, 0.0, 0.0, 0.0);

        ROS_INFO("Moving back");
        manipulator.slide_base(-obj_arm_roll_link[2], 0.0);
        manipulator.slide_base(0.0, -obj_arm_roll_link[1]);
        manipulator.move_whole_arm(0.0, 0.0, 0.0, -PI/2, 0.0);

        if (manipulator.gripper_has_object()){
            return true;
        } else {
            return false;
        }
    }

    /** Takes in an CENTROID POSITION RELATIVE TO CAMERA of an object to pick up.
    Picks up the object by extending the arm laterally and moving so that the object is within the robot's gripper.
    After this method executes, the robot will be in the same location and orientation as it was before.
    Assumes the robot is facing in the same direction as the object needing to be picked up.
    @param point: The location of the object relative to the camera frame. */
    bool pickUpPlate(geometry_msgs::Point object_location, geometry_msgs::Vector3 orientation, double radius) {

        ROS_INFO("Calculating transform for arm");
        tf::Vector3 obj_head_rgbd_sensor_link;
        obj_head_rgbd_sensor_link[0] = object_location.x;
        obj_head_rgbd_sensor_link[1] = object_location.y;
        obj_head_rgbd_sensor_link[2] = object_location.z;
        print_vector("obj_head_rgbd_sensor_link", obj_head_rgbd_sensor_link);

        tf::Vector3 obj_arm_lift_link = transform_vec(obj_head_rgbd_sensor_link, "arm_lift_link", "head_rgbd_sensor_link");
        print_vector("arm_lift_link", obj_arm_lift_link);

        ROS_INFO("Moving arm horizontally and opening gripper");
        manipulator.move_whole_arm(manipulator.get_joint_state("arm_lift_joint") + obj_arm_lift_link[2] - radius, -PI/2, 0.0, -PI/2, PI/2);
        manipulator.move_gripper(false);

        ROS_INFO("Computing amount to move base");
        tf::Vector3 obj_hand_palm_link = transform_vec(obj_arm_lift_link, "hand_palm_link", "arm_lift_link");
        print_vector("obj_hand_palm_link", obj_hand_palm_link);

        ROS_INFO("Moving laterally");
        manipulator.slide_base(0.0, -obj_hand_palm_link[1], abs(obj_hand_palm_link[1])*10.0 + 0.5); // Horizontal
        manipulator.slide_base(obj_hand_palm_link[2] + 0.01, 0.0, abs(obj_hand_palm_link[2])*10.0 + 0.5); // Forward/back TODO HACK

        ROS_INFO("Closing gripper");
        manipulator.grasp();

        manipulator.move_arm(manipulator.get_joint_state("arm_lift_joint") + OBJECT_LIFT_OFFSET, -PI/2);

        ROS_INFO("Moving back");
        manipulator.slide_base(-obj_hand_palm_link[2], 0.0, abs(obj_hand_palm_link[2])*10.0 + 0.5);
        manipulator.slide_base(0.0, obj_hand_palm_link[1], abs(obj_hand_palm_link[1])*10.0 + 0.5);

        manipulator.move_whole_arm(0.0, 0.0, 0.0, -PI/2, 0.0);

        if (manipulator.gripper_has_object()){
                return true;
        }
        else{
                   return false;
         }
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


        ROS_INFO("befor obj_base_link: %.2lf, %.2lf, %.2lf",obj_base_link[0], obj_base_link[1], obj_base_link[2]);
        tf::Vector3 obj_arm_lift_link = transform_vec(obj_base_link, "arm_lift_link", "base_link");
        ROS_INFO("atfter obj_base_link: %.2lf, %.2lf, %.2lf",obj_base_link[0], obj_base_link[1], obj_base_link[2]);
        // tf::Vector3 obj_base_link = transform_vec(obj_head_rgbd_sensor_link, "base_link", "head_rgbd_sensor_link");
        // Stretch armjkj:w
        manipulator.move_arm(manipulator.get_joint_state("arm_lift_joint") + obj_arm_lift_link[2], -PI / 2);
        double shoulder_angle = 0;
        if (obj_base_link[2] < 0){
          ROS_ERROR("Can't put down object because target location is under the ground.");
          return false;
        } else if (obj_base_link[2] <= MAX_SHOULDER_HEIGHT){
          manipulator.move_arm(manipulator.get_joint_state("arm_lift_joint") + obj_arm_lift_link[2], -PI / 2);
        } else if (obj_base_link[2] < MAX_SHOULDER_HEIGHT + ARM_LENGTH) {
          shoulder_angle = asin((obj_base_link[2] - MAX_SHOULDER_HEIGHT) / ARM_LENGTH);
          manipulator.move_arm_and_hand(MAX_ARM_LIFT_JOINT, -PI/2 + shoulder_angle, -shoulder_angle);
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

        tf::Vector3 hand_base_link = transform_vec(tf::Vector3(0,0,0), "base_link", "hand_palm_link");
        print_vector("hand_base_link", hand_base_link);

        double horizontalDistance = obj_base_link[1] - hand_base_link[1];
        double forwardDistance = obj_base_link[0] - hand_base_link[0];

        ROS_INFO("Moving laterally");
        manipulator.slide_base(0.0, horizontalDistance); // Horizontal
        manipulator.slide_base(forwardDistance, 0.0); // Forward/back

        ROS_INFO("Setting down");
        setDown();

        ROS_INFO("Opening gripper");
        manipulator.move_gripper(false);

        ROS_INFO("Moving back");
        slightlyRiseUpArm(shoulder_angle);
        manipulator.slide_base(-forwardDistance, 0.0);
        manipulator.slide_base(0.0, -horizontalDistance);

        return true;
    }

    /*
    Takes in a CENTROID POSITION RELATIVE TO DISHWASHER of where to put down something currently in the robot's hand.
    */
    bool putDownDishwasher(const geometry_msgs::Point& place_location, const string& type) {
        robot_aligner.alignAndCenterRobotToFrontObstacle();

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
        // manipulator.move_arm(manipulator.get_joint_state("arm_lift_joint") + obj_arm_lift_link[2], -PI / 2);
        double shoulder_angle = 0;
        if (obj_base_link[2] < 0) {
            ROS_ERROR("Can't put down object because target location is under the ground.");
            return false;
        } else if (obj_base_link[2] <= MAX_SHOULDER_HEIGHT) {
            manipulator.move_arm(manipulator.get_joint_state("arm_lift_joint") + obj_arm_lift_link[2], -PI / 2);
        } else if (obj_base_link[2] < MAX_SHOULDER_HEIGHT + ARM_LENGTH) {
            shoulder_angle = asin((obj_base_link[2] - MAX_SHOULDER_HEIGHT) / ARM_LENGTH);
            manipulator.move_arm_and_hand(MAX_ARM_LIFT_JOINT, -PI/2 + shoulder_angle, -shoulder_angle);
        } else {
            ROS_ERROR("Can't put down object because target location is too high.");
            return false;
        }
        ROS_INFO("Arm raised");

        ROS_INFO("hand in base_link:");
        tf::Vector3 hand_base_link = transform_vec(tf::Vector3(0,0,0), "base_link", "hand_palm_link");
        print_vector("hand_base_link", hand_base_link);

        double horizontalDistance = obj_base_link[1] - hand_base_link[1];
        double forwardDistance = obj_base_link[0] - hand_base_link[0];

        ROS_INFO("Moving %f m laterally and %f m forward", horizontalDistance, forwardDistance);
        //manipulator.slide_base(0.0, horizontalDistance, abs(horizontalDistance)*10.0 + 0.5); // Horizontal
        //manipulator.slide_base(forwardDistance, 0.0, abs(forwardDistance)*10.0 + 0.5); // Forward/back

        ROS_INFO("Setting down");
        if (type == "other") {
            // ROS_INFO("Turning wrist 180 degrees");
            // std::vector<double> current_pos = manipulator.get_trajectory_state("/hsrb/arm_trajectory_controller/query_state").response.position;
            // manipulator.move_whole_arm(current_pos[0], current_pos[1], current_pos[2], current_pos[3], PI);
            ROS_INFO("Setting down arm");
            setDown();
            ROS_INFO("Opening gripper");
            manipulator.move_gripper(false);
            // ROS_INFO("Turning wrist back to initial position");
            // manipulator.set_arm_joint_state("wrist_roll_joint", 0.0);

        } else if (type == "plate") {
            // ROS_INFO("Turning wrist 90 degrees");
            // manipulator.set_arm_joint_state("wrist_roll_joint", PI/2);
            ROS_INFO("Setting down arm");
            setDown();
            ROS_INFO("Opening gripper");
            manipulator.move_gripper(false);
            // ROS_INFO("Turning wrist back to initial position");
            // manipulator.set_arm_joint_state("wrist_roll_joint", 0.0);

        } else if (type == "cutlery") {
            ROS_INFO("Opening gripper");
            manipulator.move_gripper(false);
            ROS_INFO("Opening gripper");
            manipulator.move_gripper(false);

        } else { // cascade pod
            ROS_INFO("Setting down arm");
            setDown();
            ROS_INFO("Opening gripper");
            manipulator.move_gripper(false);
        }

        ROS_INFO("Moving back");
        slightlyRiseUpArm(shoulder_angle);
        //manipulator.slide_base(-forwardDistance, 0.0, abs(forwardDistance)*10.0 + 0.5);
        //manipulator.slide_base(0.0, -horizontalDistance, abs(horizontalDistance)*10.0 + 0.5);
        //manipulator.move_whole_arm(0.0, 0.0, -PI/2, -PI/2, 0.0);

        return true;
    }

    bool scanShelf() {
        // prepare to scan
        //manipulator.turn_base(PI/2);
        //manipulator.move_arm(0.69,0.0);
        //manipulator.move_head(-PI/2,-PI/4);

        // scan
        double waitForArmDuration = 2.0;
        manipulator.move_arm(0.75, -PI, waitForArmDuration);

        // return to home
        //manipulator.move_head(0.0,0.0);
        //manipulator.turn_base(-PI/2);
        return true;
    }
};

GraspAction * executor_ = NULL;

void pickUp(const villa_manipulation::PickUpGoalConstPtr& goal, actionlib::SimpleActionServer<villa_manipulation::PickUpAction> * as) {
    ROS_INFO_STREAM("Picking up");
    bool succeeded = executor_->pickUp(goal->object_location);
    if (succeeded) {
        as->setSucceeded();
    } else {
        as->setAborted();
    }
}

void pickUpCutlery(const villa_manipulation::PickUpCutleryGoalConstPtr& goal, actionlib::SimpleActionServer<villa_manipulation::PickUpCutleryAction> * as) {
    ROS_INFO_STREAM("Picking up cutlery");
    bool succeeded = executor_->pickUpCutlery(goal->object_location, goal->orientation);
    if (succeeded) {
        as->setSucceeded();
    } else {
        as->setAborted();
    }
}

void pickUpPlate(const villa_manipulation::PickUpPlateGoalConstPtr& goal, actionlib::SimpleActionServer<villa_manipulation::PickUpPlateAction> * as) {
    ROS_INFO_STREAM("Picking up");
    bool succeeded = executor_->pickUpPlate(goal->object_location, goal->orientation, goal->radius);
    if (succeeded) {
        as->setSucceeded();
    } else {
        as->setAborted();
    }
}

void putDown(const villa_manipulation::PutDownGoalConstPtr& goal, actionlib::SimpleActionServer<villa_manipulation::PutDownAction> * as) {
    ROS_INFO_STREAM("Putting down");
    bool succeeded = executor_->putDown(goal->place_location);
    if (succeeded) {
        as->setSucceeded();
    } else {
        as->setAborted();
    }
}

void putDownDishwasher(const villa_manipulation::PutDownDishwasherGoalConstPtr& goal, actionlib::SimpleActionServer<villa_manipulation::PutDownDishwasherAction> * as) {
    ROS_INFO_STREAM("Putting down inside the dishwasher");
    bool succeeded = executor_->putDownDishwasher(goal->place_location, goal->type.data);
    if (succeeded) {
        as->setSucceeded();
    } else {
        as->setAborted();
    }
}

void perceptionPose(const villa_manipulation::PerceptionPoseGoalConstPtr& goal, actionlib::SimpleActionServer<villa_manipulation::PerceptionPoseAction> * as) {
    ROS_INFO_STREAM("Moving to perception pose");
    bool succeeded = executor_->moveToPerceptionPose();
    if (succeeded) {
        as->setSucceeded();
    } else {
        as->setAborted();
    }
}

void scanShelf(const villa_manipulation::ScanShelfGoalConstPtr& goal, actionlib::SimpleActionServer<villa_manipulation::ScanShelfAction> * as) {
    ROS_INFO_STREAM("Scanning shelf");
    bool succeeded = executor_->scanShelf();
    if (succeeded) {
        as->setSucceeded();
    } else {
        as->setAborted();
    }
}


int main(int argc, char**argv) {
    ros::init(argc, argv, "grasp_server");

    executor_ = new GraspAction();

    executor_->initialize();

    ros::NodeHandle n;

    actionlib::SimpleActionServer<villa_manipulation::PickUpAction> pickUpServer(n, "pickUp", boost::bind(&pickUp, _1, &pickUpServer), false);
    actionlib::SimpleActionServer<villa_manipulation::PickUpCutleryAction> pickUpCutleryServer(n, "pickUpCutlery", boost::bind(&pickUpCutlery, _1, &pickUpCutleryServer), false);
    actionlib::SimpleActionServer<villa_manipulation::PickUpPlateAction> pickUpPlateServer(n, "pickUpPlate", boost::bind(&pickUpPlate, _1, &pickUpPlateServer), false);
    actionlib::SimpleActionServer<villa_manipulation::PutDownAction> putDownServer(n, "putDown", boost::bind(&putDown, _1, &putDownServer), false);
    actionlib::SimpleActionServer<villa_manipulation::PutDownDishwasherAction> putDownDishwasherServer(n, "putDownDishwasher", boost::bind(&putDownDishwasher, _1, &putDownDishwasherServer), false);
    actionlib::SimpleActionServer<villa_manipulation::PerceptionPoseAction> perceptionPoseServer(n, "perceptionPose", boost::bind(&perceptionPose, _1, &perceptionPoseServer), false);
    actionlib::SimpleActionServer<villa_manipulation::ScanShelfAction> scanShelfServer(n, "scanShelf", boost::bind(&scanShelf, _1, &scanShelfServer), false);
    pickUpServer.start();
    pickUpCutleryServer.start();
    pickUpPlateServer.start();
    putDownServer.start();
    putDownDishwasherServer.start();
    perceptionPoseServer.start();
    scanShelfServer.start();

    ROS_INFO("GRASP SERVERS ARE UP");
    ros::spin();
    return 0;
}
