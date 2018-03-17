#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/point_cloud.h>

#include <std_msgs/Time.h>
#include <sensor_msgs/PointCloud2.h>
#include <hdl_graph_slam/FloorCoeffs.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "floor_detection.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FloorDetectionNode");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");

    hdl_graph_slam::FloorDetection floor_detection(node, priv_nh);

    // handle callbacks until shut down
    ros::spin();

    return 0;

}