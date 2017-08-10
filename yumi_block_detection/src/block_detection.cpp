// Node subscibing to PrimeSense depth stream, converting from ros msg to pointCloud
// format, remove far away points and finally remove ground plane.

#include <block_detection.h>

  void BlockDetection::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
  {
      // Conversion from ros msg to pointCloud
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*input,pcl_pc2);
      pcl::fromPCLPointCloud2(pcl_pc2,*(this->cloud));

      //ROS_INFO_STREAM_NAMED("block_detection", "size: " << (this->cloud)->points.size());

      pub.publish (cloud);
  }

  void BlockDetection::blockCluster()
  {
    ROS_INFO_STREAM_NAMED("block_detection", "Inside Cluster - ");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (this->cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      ROS_INFO_STREAM_NAMED("block_detection", "Inside while - ");
      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
    }

    ROS_INFO_STREAM_NAMED("block_detection", "in cloud size - " << cloud_filtered->size ());

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    ROS_INFO_STREAM_NAMED("block_detection", "cloud size - " << cluster_indices.size ());

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      ROS_INFO_STREAM_NAMED("block_detection", "Cluster size " << j);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

      // Change to true, to save each cluster as indiviusla .pcd file
      if (true)
      {
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
      }

      (this->blocks).push_back(cloud_cluster);
      j++;
    }
  }

  PointCloud::Ptr BlockDetection::removeGroundPlane()
  {
    // Remove far away points.
    PointCloud::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < this->cloud->points.size (); ++i)
    {
      if (this->cloud->points[i].z < 1.2)
        filteredCloud->push_back (pcl::PointXYZ (this->cloud->points[i].x,
                                                 this->cloud->points[i].y,
                                                 this->cloud->points[i].z));
    }

    // Find plane in pointcloud
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (filteredCloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return(filteredCloud);
    }

    pcl::PointIndices::Ptr outliers (new pcl::PointIndices);
    PointCloud::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Initializing with true will allow us to extract the removed indices
    pcl::ExtractIndices<pcl::PointXYZ> eifilter (true);
    eifilter.setInputCloud (filteredCloud);
    eifilter.setIndices (inliers);
    eifilter.filter (*tempCloud);
    // Indexes all points of cloud_in that are not indexed by inliers
    eifilter.getRemovedIndices (*outliers);

    PointCloud::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < outliers->indices.size (); ++i) {
      outCloud->push_back (pcl::PointXYZ (filteredCloud->points[outliers->indices[i]].x,
                                       filteredCloud->points[outliers->indices[i]].y,
                                       filteredCloud->points[outliers->indices[i]].z));
    }
    // Everything but the ground plane
    return(outCloud);
  }

  PointCloud::Ptr BlockDetection::segmentPlanes()
  {
    ROS_INFO_STREAM_NAMED("block_detection", "Entered segPlane ");
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);


    seg.setInputCloud ((this->cloud));
    seg.segment (*inliers, *coefficients);

    PointCloud::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return(plane);
    }
/*
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    ROS_INFO_STREAM_NAMED("block_detection", "Index size " << inliers->indices.size ());
*/
    for (size_t i = 0; i < inliers->indices.size (); ++i) {
      plane->push_back (pcl::PointXYZ (cloud->points[inliers->indices[i]].x,
                                       cloud->points[inliers->indices[i]].y,
                                       cloud->points[inliers->indices[i]].z));
    }
    return(plane);
  }

  int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "block_detection");
    ros::Subscriber sub;

    // PCL visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    //viewer->addCoordinateSystem (1.0);

    BlockDetection detector;

    while(ros::ok())
    {
      ros::spinOnce ();
      sub = detector.nh.subscribe("/camera/depth/points", 1, &BlockDetection::cloud_cb, &detector);

      if ((detector.cloud)->points.size() > 0)
      {
        PointCloud::Ptr filCloud = detector.removeGroundPlane();
        // detector.cloud = filCloud;
        //detector.blockCluster();

        // Save to pcd file
        pcl::PCDWriter writer;
        std::stringstream ss;
        ss << "thin_cuboid2.pcd"; // 3 worked only for 1, 4th found skewed, 5th stacked doesnt work at all
        writer.write<pcl::PointXYZ> (ss.str (), *filCloud, false); //*

        viewer->removeAllPointClouds();
        viewer->addPointCloud<pcl::PointXYZ> (detector.cloud, "sample cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb (filCloud, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ> (filCloud, rgb, "filtered cloud");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->spinOnce(100);
      }
    }
    return 0;
  }
