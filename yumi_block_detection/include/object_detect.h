#include <ros/ros.h>
#include <limits>
#include <fstream>
#include <vector>
#include <math.h>
#include <Eigen/Core>
// This header allows you to publish and subscribe
// pcl::PointCloud<T> objects as ROS messages.
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
// Plane Segmentation
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

// Custom messages
#include "yumi_block_detection/objectCentroid.h"
#include "yumi_block_detection/objects.h"
// Service file
#include "yumi_block_detection/objectLocation.h"

typedef pcl::PointXYZRGB PointTypeColor;
typedef pcl::PointXYZINormal PointTypeFull;
typedef pcl::PointCloud<PointTypeColor> PointCloud;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

class BlockDetect
{
public:

  PointCloud::Ptr curr_cloud;
  ros::NodeHandle nh;
  std::string filename;
  // PCL visualizer
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  BlockDetect() {

    // Topic you want to publish
    pub = nh.advertise<PointCloud>("/yumi_primesense_data", 1);

    // Input point cloud
    curr_cloud.reset(new PointCloud);
  }

  void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
  bool srv_cb(yumi_block_detection::objectLocation::Request  &req,
         yumi_block_detection::objectLocation::Response &res);
  PointCloud::Ptr removeGroundPlane();
  yumi_block_detection::objects startRoutine();

private:
  ros::Publisher pub;
};//End of class
