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

namespace hdl_graph_slam {

    class FloorDetectionNodelet : public nodelet::Nodelet {
    public:

      FloorDetectionNodelet() {}
      ~FloorDetectionNodelet() {}


    private:
        virtual void onInit()
        {
            floor_detection.reset(new FloorDetection(getNodeHandle(), getPrivateNodeHandle()));
        }

    private:
        boost::shared_ptr<FloorDetection> floor_detection;

    };

} // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::FloorDetectionNodelet, nodelet::Nodelet)
