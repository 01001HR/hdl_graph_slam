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

namespace hdl_graph_slam {

    class ScanMatchingOdometryNodelet : public nodelet::Nodelet
    {
    public:
      ScanMatchingOdometryNodelet() {}
      ~ScanMatchingOdometryNodelet() {}


    private:
        virtual void onInit()
        {
            odom.reset(new ScanMatchingOdometry(getNodeHandle(), getPrivateNodeHandle()));
        }

    private:
        boost::shared_ptr<ScanMatchingOdometry> odom;

    };

} // namespace hdl_graph_slam


// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::ScanMatchingOdometryNodelet, nodelet::Nodelet)
