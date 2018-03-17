#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/registrations.hpp>

#include "scan_matching_odometry.cpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ScanMatchingOdometryNode");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");

    hdl_graph_slam::ScanMatchingOdometry odom(node, priv_nh);

    // handle callbacks until shut down
    ros::spin();

    return 0;

}