#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "prefiltering.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PrefilteringNode");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");

    hdl_graph_slam::Prefiltering prefilter(node, priv_nh);

    // handle callbacks until shut down
    ros::spin();

    return 0;

}
