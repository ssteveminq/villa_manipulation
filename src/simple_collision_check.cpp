#include <tf2_ros/Transform.h>
#include <villa_manipulation/transform_utils.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Return true if there are fewer than tolerance collisions in the range start-finish, where start and finish are the height values, as passed to the arm. This assumes the arm is at PI/2.
bool simple_collision_check(const sensor_msgs::PointCloud2ConstPtr& msg, float start = 0.0, float finish = 0.69, int tolerance = 50) {
  ROS_INFO_STREAM("Checking for collisions...");
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*msg, cloud);
  
  tf2::Vector3 zeros; // so we won't see our own arm
  zeros.setZero();
  
  // Check that no points are within the following bounds in frame base_link
  float x_min = 0; // NEED THIS
  float x_max = 0.05 + transform_vec(zeros,"base_link","hand_palm_link")[0];
  float y_min = -0.1; // totally random guess about width of arm and location of base_link origin
  float y_max = 0.2; // a little larger, since the arm is slightly offset to the left?
  float z_min = 0.339999 + (start<finish)?start:finish - 0.03; // +- 0.03 since the hand has height
  float z_max = 0.339999 + (start<finish)?finish:start + 0.03;

  // The checking will be done in base_link, so this will be applied to all the points before checking
  tf2::Transform transform = tf2::get_transform("base_link","head_rgbd_sensor_link");
  
  int cnt = 0;
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++) {
    tf2::Vector3 camera_link_pt (it->x,it->y,it->z);
    tf2::Vector3 pt = transform(camera_link_pt);
    if (x_min<pt.x and pt.x<x_max and y_min<pt.y and pt.y<y_max and z_min<pt.z and pt.z<z_max) {
      cnt++;
      if (cnt > tolerance) return false;
    }
  }
  return true;
}

void pc_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
  ROS_INFO_STREAM("Collision check: " << simplie_collision_check(msg));
}

int main(int argc, char** argv) {
  ros::init(argc,argv,"simple_collision_check");
  ros::NodeHandle nh_;
  ros::Subscriber sub = nh_.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",1,pc_cb);

  ros::Rate r(1);
  while (ros.ok()) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
