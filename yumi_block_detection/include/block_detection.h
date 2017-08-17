#include <ros/ros.h>
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
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class BlockDetection
{
public:
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloud::Ptr cloud;
  ros::NodeHandle nh;
  std::vector<PointCloud::Ptr> blocks;

  BlockDetection() {

    //Topic you want to publish
    pub = nh.advertise<PointCloud>("/yumi_primesense_data", 1);

    //Topic you want to subscribe
    //sub = nh.subscribe("/camera/depth/points", 1, &BlockDetection::cloud_cb, this);

        // Input point cloud
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  }

  void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
  //PointCloud::Ptr filterCloud();
  PointCloud::Ptr segmentPlanes();
  PointCloud::Ptr removeGroundPlane();
  void blockCluster();
  //PointCloud::Ptr filterCloud(PointCloud::Ptr inputCloud);

private:
  ros::Publisher pub;


};//End of class
