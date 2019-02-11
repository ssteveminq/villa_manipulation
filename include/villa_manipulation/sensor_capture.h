#ifndef SENSOR_CAPTURE
#define SENSOR_CAPTURE

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

class ForceSensorCapture {
public:
    ForceSensorCapture() : force_data_(), wrist_wrench_sub_(), nh_() {
        ROS_INFO("Waiting on wrist force subscriber...");
        // Subscribe to force torque sensor data

        std::string ft_sensor_topic = "/hsrb/wrist_wrench/raw";
        wrist_wrench_sub_ = nh_.subscribe(ft_sensor_topic, 1, &ForceSensorCapture::forceSensorCallback, this);

        // Wait for connection
        if (!ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(ft_sensor_topic, ros::Duration(10.0))) {
			ROS_ERROR("Timeout exceeded while waiting for message on topic %s", ft_sensor_topic.c_str());
			exit(EXIT_FAILURE);
        }
    }

    void getCurrentForce(geometry_msgs::Vector3& force_data) const {
		// Spin FtSensorCb function once
		ros::spinOnce();
		force_data = force_data_;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber wrist_wrench_sub_;
    geometry_msgs::Vector3 force_data_;

    void forceSensorCallback(const geometry_msgs::WrenchStampedConstPtr& data) {
        force_data_ = data->wrench.force;
    }

};

class OdometryCapture {
public:
    OdometryCapture() : base_location_(), base_orientation_(), odom_sub_(), nh_() {
        ROS_INFO("Waiting on base odom subscriber...");
        std::string odom_sensor_topic = "/hsrb/odom";
        odom_sub_ = nh_.subscribe(odom_sensor_topic, 1, &OdometryCapture::odometryCallback, this);

        if (!ros::topic::waitForMessage<nav_msgs::Odometry>(odom_sensor_topic, ros::Duration(10.0))) {
            ROS_ERROR("Timeout");
            exit(EXIT_FAILURE);
        }
    }

    void getCurrentOdometry(geometry_msgs::Point& location, geometry_msgs::Quaternion& orientation) const {
        ros::spinOnce();
        location = base_location_;
        orientation = base_orientation_;
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    geometry_msgs::Point base_location_;
    geometry_msgs::Quaternion base_orientation_;

    void odometryCallback(const nav_msgs::OdometryConstPtr& data) {
        base_location_ = data->pose.pose.position;
        base_orientation_ = data->pose.pose.orientation;
    }
};

#endif