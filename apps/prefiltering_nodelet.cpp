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
namespace hdl_graph_slam {

class PrefilteringNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;

  PrefilteringNodelet() {}
  ~PrefilteringNodelet() {}


private:
    virtual void onInit()
    {
        prefilter.reset(new Prefiltering(getNodeHandle(), getPrivateNodeHandle()));
    }

private:
    boost::shared_ptr<Prefiltering> prefilter;

};

} // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::PrefilteringNodelet, nodelet::Nodelet)
