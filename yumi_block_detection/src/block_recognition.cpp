#include <block_recognition.h>

  void BlockRecognition::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
  {
      // Conversion from rosmsg to pointCloud
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*input,pcl_pc2);
      pcl::fromPCLPointCloud2(pcl_pc2,*(this->curr_cloud));

      pub.publish (this->curr_cloud);
  }

  // convert RGB to CIELAB
  Eigen::Vector3f RGB2Lab (const Eigen::Vector3i& colorRGB)
  {
    // for sRGB   -> CIEXYZ see http://www.easyrgb.com/index.php?X=MATH&H=02#text2
    // for CIEXYZ -> CIELAB see http://www.easyrgb.com/index.php?X=MATH&H=07#text7

    double R, G, B, X, Y, Z;

    // map sRGB values to [0, 1]
    R = colorRGB[0] / 255.0;
    G = colorRGB[1] / 255.0;
    B = colorRGB[2] / 255.0;

    // linearize sRGB values
    if (R > 0.04045)
      R = pow ( (R + 0.055) / 1.055, 2.4);
    else
      R = R / 12.92;

    if (G > 0.04045)
      G = pow ( (G + 0.055) / 1.055, 2.4);
    else
      G = G / 12.92;

    if (B > 0.04045)
      B = pow ( (B + 0.055) / 1.055, 2.4);
    else
      B = B / 12.92;

    // postponed:
    //    R *= 100.0;
    //    G *= 100.0;
    //    B *= 100.0;

    // linear sRGB -> CIEXYZ
    X = R * 0.4124 + G * 0.3576 + B * 0.1805;
    Y = R * 0.2126 + G * 0.7152 + B * 0.0722;
    Z = R * 0.0193 + G * 0.1192 + B * 0.9505;

    // *= 100.0 including:
    X /= 0.95047;  //95.047;
    //    Y /= 1;//100.000;
    Z /= 1.08883;  //108.883;

    // CIEXYZ -> CIELAB
    if (X > 0.008856)
      X = pow (X, 1.0 / 3.0);
    else
      X = 7.787 * X + 16.0 / 116.0;

    if (Y > 0.008856)
      Y = pow (Y, 1.0 / 3.0);
    else
      Y = 7.787 * Y + 16.0 / 116.0;

    if (Z > 0.008856)
      Z = pow (Z, 1.0 / 3.0);
    else
      Z = 7.787 * Z + 16.0 / 116.0;

    Eigen::Vector3f colorLab;
    colorLab[0] = static_cast<float> (116.0 * Y - 16.0);
    colorLab[1] = static_cast<float> (500.0 * (X - Y));
    colorLab[2] = static_cast<float> (200.0 * (Y - Z));

    return colorLab;
  }

  PointCloud::Ptr BlockRecognition::removeGroundPlane()
  {
    // Remove far away points.
    PointCloud::Ptr filteredCloud(new PointCloud);
    for (size_t i = 0; i < this->curr_cloud->points.size (); ++i)
    {
      if (this->curr_cloud->points[i].z < 1.2)
      {
        pcl::PointXYZRGB point = pcl::PointXYZRGB(this->curr_cloud->points[i].r,
        this->curr_cloud->points[i].g, this->curr_cloud->points[i].b);
        point.x = this->curr_cloud->points[i].x;
        point.y = this->curr_cloud->points[i].y;
        point.z = this->curr_cloud->points[i].z;
        filteredCloud->push_back (point);
      }
    }

    // Find plane in pointcloud
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
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
    PointCloud::Ptr tempCloud (new PointCloud);

    // Initializing with true will allow us to extract the removed indices
    pcl::ExtractIndices<pcl::PointXYZRGB> eifilter (true);
    eifilter.setInputCloud (filteredCloud);
    eifilter.setIndices (inliers);
    eifilter.filter (*tempCloud);
    // Indexes all points of cloud_in that are not indexed by inliers
    eifilter.getRemovedIndices (*outliers);

    PointCloud::Ptr outCloud(new PointCloud);

    for (size_t i = 0; i < outliers->indices.size (); ++i)
    {
      pcl::PointXYZRGB point = pcl::PointXYZRGB(filteredCloud->points[outliers->indices[i]].r,
      filteredCloud->points[outliers->indices[i]].b, filteredCloud->points[outliers->indices[i]].g);
      point.x = filteredCloud->points[outliers->indices[i]].x;
      point.y = filteredCloud->points[outliers->indices[i]].y;
      point.z = filteredCloud->points[outliers->indices[i]].z;
      outCloud->push_back (point);
    }
    // Everything but the ground plane
    return(outCloud);
  }

  PointCloud::Ptr BlockRecognition::segmentColorClusters()
  {
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree =
    boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCloud (new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBtoXYZHSV(*(this->curr_cloud), *hsvCloud);

    PointCloud::Ptr inputCloud = this->curr_cloud;
    /*
    // HSV
    for (size_t i = 0; i < inputCloud->points.size (); ++i)
    {
      pcl::PointXYZHSV hsv;
      pcl::PointXYZRGBtoXYZHSV (inputCloud->points[i], hsv);

      inputCloud->points[i].r = hsv.h;
      inputCloud->points[i].g = hsv.s;
      inputCloud->points[i].b = hsv.v;
    }
    */
    // LAB
    for (size_t i = 0; i < inputCloud->points.size (); ++i)
    {
      Eigen::Vector3i colorRGB = {inputCloud->points[i].r,
                                  inputCloud->points[i].g,
                                  inputCloud->points[i].b};

      Eigen::Vector3f lab = RGB2Lab (colorRGB);

      inputCloud->points[i].r = 0; //lab[0];
      inputCloud->points[i].g = lab[1];
      inputCloud->points[i].b = lab[2];
    }

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (this->curr_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (this->curr_cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (10); //6 //35 //10
    reg.setRegionColorThreshold (15); //5  //40 //10
    reg.setMinClusterSize (600);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    this->clusters = clusters;
    ROS_INFO_STREAM_NAMED("block_recognition", "# of clusters" << clusters.size());

    PointCloud::Ptr colored_cloud = reg.getColoredCloud ();
    return (colored_cloud);
  }

  bool BlockRecognition::blockCorrespondence(PointCloud::Ptr segment)
  {
    //Algorithm params
    bool show_keypoints_ (true);
    bool show_correspondences_ (true);
    bool use_cloud_resolution_ (false);
    bool use_hough_ (true);
    float model_ss_ (0.001f); // 0.01f //0.001
    float scene_ss_ (0.01f); // 0.03f //0.01
    float rf_rad_ (0.015f);
    float descr_rad_ (0.02f);
    float cg_size_ (0.01f);
    float cg_thresh_ (5.0f);

    std::string model_filename_ = "cylinder.pcd"; // start with just one shape
    std::string scene_filename_ = "cloud_cluster_scene_close.pcd"; // use input from subsciber
    bool found (false);

    pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

    scene = segment;

    //  Load clouds
    if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
    {
      std::cout << "Error loading model cloud." << std::endl;
    }
    /*
    if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
    {
      std::cout << "Error loading scene cloud." << std::endl;
    }
    */

    //  Compute Normals
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (model);
    norm_est.compute (*model_normals);

    norm_est.setInputCloud (scene);
    norm_est.compute (*scene_normals);

    //  Downsample Clouds to Extract keypoints
    pcl::UniformSampling<PointType> uniform_sampling;
    /*
    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (model_ss_);
    uniform_sampling.filter (*model_keypoints);
    std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;
    */
    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (scene_ss_);
    uniform_sampling.filter (*scene_keypoints);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

    //scene_keypoints = scene;
    model_keypoints = model;

    //  Compute Descriptor for keypoints
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descr_rad_);

    descr_est.setInputCloud (model_keypoints);
    descr_est.setInputNormals (model_normals);
    descr_est.setSearchSurface (model);
    descr_est.compute (*model_descriptors);

    descr_est.setInputCloud (scene_keypoints);
    descr_est.setInputNormals (scene_normals);
    descr_est.setSearchSurface (scene);
    descr_est.compute (*scene_descriptors);

    // Find Model-Scene Correspondences with KdTree
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud (model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < scene_descriptors->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);
      if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
      {
        continue;
      }
      int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
      if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
      }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

    //  Actual Clustering
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  Using Hough3D
    if (use_hough_)
    {
      //  Compute (Keypoints) Reference Frames only for Hough
      pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
      pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

      pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
      rf_est.setFindHoles (true);
      rf_est.setRadiusSearch (rf_rad_);

      rf_est.setInputCloud (model_keypoints);
      rf_est.setInputNormals (model_normals);
      rf_est.setSearchSurface (model);
      rf_est.compute (*model_rf);

      rf_est.setInputCloud (scene_keypoints);
      rf_est.setInputNormals (scene_normals);
      rf_est.setSearchSurface (scene);
      rf_est.compute (*scene_rf);

      //  Clustering
      pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
      clusterer.setHoughBinSize (cg_size_);
      clusterer.setHoughThreshold (cg_thresh_);
      clusterer.setUseInterpolation (true);
      clusterer.setUseDistanceWeight (false);

      clusterer.setInputCloud (model_keypoints);
      clusterer.setInputRf (model_rf);
      clusterer.setSceneCloud (scene_keypoints);
      clusterer.setSceneRf (scene_rf);
      clusterer.setModelSceneCorrespondences (model_scene_corrs);

      //clusterer.cluster (clustered_corrs);
      clusterer.recognize (rototranslations, clustered_corrs);
    }
    else // Using GeometricConsistency
    {
      pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
      gc_clusterer.setGCSize (cg_size_);
      gc_clusterer.setGCThreshold (cg_thresh_);

      gc_clusterer.setInputCloud (model_keypoints);
      gc_clusterer.setSceneCloud (scene_keypoints);
      gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

      //gc_clusterer.cluster (clustered_corrs);
      gc_clusterer.recognize (rototranslations, clustered_corrs);
    }

    //  Output results
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    if (rototranslations.size() > 0)
      found = true;
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
      std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
      std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

      // Print the rotation matrix and translation vector
      Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
      Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

      printf ("\n");
      printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
      printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
      printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
      printf ("\n");
      printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    }

    //  Visualization
    pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
    viewer.addPointCloud (scene, "scene_cloud");

    pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

    if (show_correspondences_ || show_keypoints_)
    {
      //  We are translating the model so that it doesn't end in the middle of the scene representation
      pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
      pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

      pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
      viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
    }

    if (show_keypoints_)
    {
      pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
      viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

      pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
      viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
    }

    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
      pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
      pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

      std::stringstream ss_cloud;
      ss_cloud << "instance" << i;

      pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
      viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

      if (show_correspondences_)
      {
        for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
        {
          std::stringstream ss_line;
          ss_line << "correspondence_line" << i << "_" << j;
          PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
          PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

          //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
          viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
        }
      }
    }
    viewer.spinOnce(3000);
    return(found);
  }

  double BlockRecognition::icpCluster(PointCloud::Ptr segment)
  {
    PointCloud::Ptr cloud_in (new PointCloud);
    PointCloud::Ptr cloud_out (new PointCloud);

    cloud_out = segment;

    std::string model_filename_ = "cylinder.pcd"; // start with just one shape
    if (pcl::io::loadPCDFile (model_filename_, *cloud_in) < 0)
    {
      std::cout << "Error loading model cloud." << std::endl;
    }

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<PointType> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    return(icp.getFitnessScore());
  }

  int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "block_recognition");
    ros::Subscriber sub;

    bool first = true;

    // PCL visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    BlockRecognition detector;

    while(ros::ok())
    {
      ros::spinOnce ();
      sub = detector.nh.subscribe("/camera/depth_registered/points", 1, &BlockRecognition::cloud_cb, &detector);

      if ((detector.curr_cloud)->points.size() > 0)
      {
        PointCloud::Ptr filCloud = detector.removeGroundPlane();
        detector.curr_cloud = filCloud;
        //PointCloud::Ptr segCloud = detector.segmentColorClusters();

/*
        // Send individual cluster to see what shape it is
        pcl::PointCloud<PointType>::Ptr segment (new pcl::PointCloud<PointType> ());
        std::vector< pcl::PointIndices >::iterator i_segment;
        ROS_INFO_STREAM_NAMED("block_recognition", "*********** NEW FRAME ***********");
        for (i_segment = detector.clusters.begin (); i_segment != detector.clusters.end (); i_segment++)
        {
          std::vector<int>::iterator i_point;
          for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
          {
            int index;
            index = *i_point;
            pcl::PointXYZRGB point = pcl::PointXYZRGB(segCloud->points[index].r,
            segCloud->points[index].g, segCloud->points[index].b);
            point.x = segCloud->points[index].x;
            point.y = segCloud->points[index].y;
            point.z = segCloud->points[index].z;
            segment->push_back (point);
          }
          ROS_INFO_STREAM_NAMED("block_recognition", "Before calling blockCorrespondence");
          // Once segmented, try identifying what object each cluster represents
          if (detector.icpCluster(segment)) //(detector.blockCorrespondence(segment))
            break;
*/
          if (first)
          {
            // Save to pcd file
            pcl::PCDWriter writer;
            std::stringstream ss;
            ss << "color_scene_table1.pcd"; // 3 worked only for 1, 4th found skewed, 5th stacked doesnt work at all
            writer.write<pcl::PointXYZRGB> (ss.str (), *filCloud, false); //*
            first = false;
          }
      //}
        viewer->removeAllPointClouds();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(detector.curr_cloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (detector.curr_cloud, color, "sample cloud");

        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb (segCloud, 255, 0, 0);
        //viewer->addPointCloud<pcl::PointXYZRGB> (segCloud, rgb, "filtered cloud");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->spinOnce(100);
      }
    }
    return 0;
  }
