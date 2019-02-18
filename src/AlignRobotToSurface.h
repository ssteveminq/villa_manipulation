#ifndef ALIGN_ROBOT_TO_SURFACE_H
#define ALIGN_ROBOT_TO_SURFACE_H

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <numeric>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include "manipulation_controller.h"

using namespace std;

const bool DEBUG = true;

const string __WORLD_FRAME = "/map";
const string __ROBOT_FRAME = "/base_link";

const int __OUT_OF_BOUNDS = -1;
const int __NOT_OCCUPIED = 0;
const int __OCCUPIED = 1;
const double __PI = 3.1416;
const double __SURFACE_DEVIATION_THRESHOLD = 0.07; // m

class AlignRobotToSurface {
protected:
    ros::NodeHandle& nh_;
    tf::TransformListener transform_listener_;
    Manipulator manipulator_;
    ros::Subscriber obstacle_map_sub_;
    ros::Publisher marker_pub_;
	nav_msgs::OccupancyGridConstPtr obstacle_map_ = NULL;

    bool received_obstacle_map(){
        if (obstacle_map_ == NULL){
            ros::Duration(3.0).sleep();
            return obstacle_map_ != NULL;
        }
        return true;
    }

    int get_cell_occupancy(int i, int j) {
        int width = obstacle_map_->info.width;
        int height = obstacle_map_->info.height;
        int n = i + width*j;
        if (n < 0 or n >= width*height or n%width == 0) return __OUT_OF_BOUNDS;
        return obstacle_map_->data[n] > 50 ? __OCCUPIED : __NOT_OCCUPIED;
    }

    double get_world_to_robot_angle(){
        tf::StampedTransform transform;
        try {
            transform_listener_.waitForTransform(__ROBOT_FRAME, __WORLD_FRAME, obstacle_map_->info.map_load_time, ros::Duration(10.0) );
            transform_listener_.lookupTransform(__ROBOT_FRAME, __WORLD_FRAME, obstacle_map_->info.map_load_time, transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
        return transform.getRotation().getAngle();
    }

    int round_double_to_int(double d){
        return round(d) + 0.0001;
    }
// Line robot_frame_y_axis line in "world orientation and robot origin frame": a*x + b*y = 0
public:
    AlignRobotToSurface(ros::NodeHandle& node_handle) :
        nh_(node_handle),
        manipulator_(nh_)
    {
        obstacle_map_sub_ = nh_.subscribe("/dynamic_obstacle_map_ref", 1, &AlignRobotToSurface::obstacle_map_callback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("surface_line", 10);
    }

    void obstacle_map_callback(const nav_msgs::OccupancyGridConstPtr& obstacle_map_msg){
        obstacle_map_ = obstacle_map_msg;
    }

    // @param distance_to_obstacle: final distance from base_link origin to the obstacle in meters
    // Pre: robot has a linear obstacle in front whose right and left bounds can be seen (eg a wardrobe, a fridge)
    // Post: robot has moved in a way such that it is perpendicular and centered to the obstacle at a distance of "distance_to_obstacle"
    bool alignAndCenterRobotToFrontObstacle(double distance_to_obstacle = 0.1){
        if (not received_obstacle_map()){
            ROS_ERROR("No dynamic map received");
            return false;
        }

        double world_to_robot_angle = get_world_to_robot_angle();
        int width = obstacle_map_->info.width;
        int height = obstacle_map_->info.height;
        double resolution = obstacle_map_->info.resolution;
        int radius = min(width/2, height/2);
        if (radius < 15) return false; // Too small for an obstacle map
        double x_i_incr = cos(world_to_robot_angle - __PI/2); // in cells
        double y_i_incr = sin(world_to_robot_angle - __PI/2); // in cells
        double x_j_incr = cos(world_to_robot_angle); // in cells
        double y_j_incr = sin(world_to_robot_angle); // in cells

        vector<double> distances_to_robot_frame_y_axis(radius*2+1);
        // Line robot_frame_y_axis line in "world orientation and robot origin frame": a*x + b*y [+ c] = 0
        double a = 1;
        double b = tan(world_to_robot_angle);
        // c = 0
        for (int i=-radius; i<radius; ++i){
            int j=0;
            char cell_occupancy = get_cell_occupancy(width/2 + round_double_to_int(i*x_i_incr + j*x_j_incr), height/2 + round_double_to_int(i*y_i_incr + j*y_j_incr));
            while (cell_occupancy == __NOT_OCCUPIED and j<radius){
                ++j;
                cell_occupancy = get_cell_occupancy(width/2 + round_double_to_int(i*x_i_incr + j*x_j_incr), height/2 + round_double_to_int(i*y_i_incr + j*y_j_incr));
            }
            if (cell_occupancy == __OUT_OF_BOUNDS){
                distances_to_robot_frame_y_axis[i+radius] = -1.0;
            } else { // cell_occupancy == __OCCUPIED
                double x = round_double_to_int(j*x_j_incr); // in cells
                double y = round_double_to_int(j*y_j_incr); // in cells
                distances_to_robot_frame_y_axis[i+radius] = sqrt(pow(x, 2) + pow(y,2)) - distance_to_obstacle/resolution; // in cells
            }
        }

        // Calculate avg distance
        ROS_INFO("Distances of points to base_link y axis (in cells; 1 cell = 5cm)");
        double average_distance = 0.0;
        for (int i=-3; i<=3; ++i){
            average_distance += distances_to_robot_frame_y_axis[i+radius]; // in cells
            cout << distances_to_robot_frame_y_axis[i+radius] << ", ";
        }
        average_distance = resolution * average_distance / 7.0; // in m
        cout << endl;
        ROS_INFO("Average distance to surface: %f m", average_distance);

        // Calculating beginning and end of obstacle surface
        int min_i = 42; // the answer to everything
        for (int i=0; i<radius and min_i==42; --i){
            if (abs(resolution /* m/cell */ * distances_to_robot_frame_y_axis[radius + i] /* cell */ - average_distance /* m */) > __SURFACE_DEVIATION_THRESHOLD /* m */){
                min_i = i+1;
            }
        }
        if (min_i == 42) min_i = -radius+1;

        int max_i = -42; // the negative version of the answer to everything
        for (int i=0; i<radius and max_i==-42; ++i){
            if (abs(resolution /* m/cell */ * distances_to_robot_frame_y_axis[radius + i] /* cell */ - average_distance /* m */) > __SURFACE_DEVIATION_THRESHOLD /* m */){
                max_i = i-1;
            }
        }
        if (max_i == -42) max_i = radius-1;

        double displacement = -resolution * (max_i+min_i)/2.0;
        ROS_INFO("min_i: %i; max_i: %i; displacing %f metres to the right", min_i, max_i, -displacement);
        manipulator_.move_base_relative(0.0, displacement, 0.0, abs(displacement)*10.0 + 0.5);
        ROS_INFO("mov horitzontal feta");

        // Go forward (average_distance meters)
        displacement = resolution * (distances_to_robot_frame_y_axis[radius + (max_i+min_i)/2] + distances_to_robot_frame_y_axis[radius + (max_i+min_i+1)/2]) / 2;
        ROS_INFO("Moving forward %f metres.", displacement);
        manipulator_.move_base_relative(displacement, 0.0, 0.0, abs(displacement)*10.0 + 0.5);
        ROS_INFO("avancament feta");

        // Calculate slope of the obstacle surface
        vector<double> x_values(7); // in cells
        for (int i=0; i<7; ++i) x_values[i] = i-3;
        vector<double> y_values(7); // in cells
        for (int i=0; i<7; ++i) y_values[i] = distances_to_robot_frame_y_axis[radius+i-3];
        const auto n    = x_values.size();
        const auto s_x  = std::accumulate(x_values.begin(), x_values.end(), 0.0);
        const auto s_y  = std::accumulate(y_values.begin(), y_values.end(), 0.0);
        const auto s_xx = std::inner_product(x_values.begin(), x_values.end(), x_values.begin(), 0.0);
        const auto s_xy = std::inner_product(x_values.begin(), x_values.end(), y_values.begin(), 0.0);
        double slope    = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x); // in cells/cells = m/m
        ROS_INFO("Slope of the surface is %f", slope);

        // Publish obstacle detection line
        visualization_msgs::Marker line;
        line.header.frame_id = "/base_link";
        line.header.stamp = ros::Time::now();
        line.ns = "align_robot_to_surface";
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.id = 0;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.scale.x = 0.1;
        line.color.a = 1.0;
        line.color.b = 1.0;
        geometry_msgs::Point p, q;
        p.x = resolution*min_i; p.y = resolution * distances_to_robot_frame_y_axis[radius + min_i]; p.z = 0.0;
        line.points.push_back(p);
        q.x = resolution*max_i; q.y = resolution * distances_to_robot_frame_y_axis[radius + max_i]; q.z = 0.0;
        line.points.push_back(q);
        // line.points.push_back(geometry_msgs::Point(resolution*min_i, resolution * distances_to_robot_frame_y_axis[radius + min_i], 0.0));
        // line.points.push_back(geometry_msgs::Point(resolution*max_i, resolution * distances_to_robot_frame_y_axis[radius + max_i], 0.0));

        // Turn around to face obstacle
        ROS_INFO("Turning robot %f  degrees.", atan(slope)*180/__PI);
        manipulator_.move_base_relative(0.0, 0.0, atan(slope));
        ROS_INFO("rotacio feta");
    }
};

#endif
