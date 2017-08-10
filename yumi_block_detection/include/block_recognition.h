#include <ros/ros.h>
#include <vector>
// This header allows you to publish and subscribe
// pcl::PointCloud<T> objects as ROS messages.
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// Plane Segmentation
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
// Eucledian Clustering
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
// Correspondence
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
// ICP`
#include <pcl/registration/icp.h>
// For LAB conversion
#include <limits>
#include <pcl/point_types_conversion.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

class BlockRecognition
{
public:

  PointCloud::Ptr curr_cloud;
  ros::NodeHandle nh;
  std::vector <pcl::PointIndices> clusters;

  BlockRecognition() {

    //Topic you want to publish
    pub = nh.advertise<PointCloud>("/yumi_primesense_data", 1);

    //Topic you want to subscribe
    //sub = nh.subscribe("/camera/depth/points", 1, &BlockRecognition::cloud_cb, this);

    // Input point cloud
    curr_cloud.reset(new PointCloud);
  }

  void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
  PointCloud::Ptr removeGroundPlane();
  PointCloud::Ptr segmentColorClusters();
  bool blockCorrespondence(PointCloud::Ptr segment);
  double icpCluster(PointCloud::Ptr segment);

private:
  ros::Publisher pub;

};//End of class SubscribeAndPublish
