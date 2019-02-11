#include <villa_manipulation/transform_utils.h>
#include <tf/transform_listener.h>

/* Gets the transform from source to target */
tf::StampedTransform get_transform(const std::string target, const std::string source) {
    tf::TransformListener listener;

    tf::StampedTransform transform;

    try {
        listener.waitForTransform(target, source, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(target, source, ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }

    return transform;
}

/* Looks up the transformation and applies it to a vector. */
tf::Vector3 transform_vec(tf::Vector3 vector, const std::string target, const std::string source) {
    tf::StampedTransform transform = get_transform(target, source);
    return transform(vector);
}
