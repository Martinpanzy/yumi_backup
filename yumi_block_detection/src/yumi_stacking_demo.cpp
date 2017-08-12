#include <limits>
#include <fstream>
#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

typedef pcl::PointXYZRGB PointTypeColor;
typedef pcl::PointXYZINormal PointTypeFull;

class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.005f),
      feature_radius_ (0.005f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.01f), //0.05
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (2000)
    {
      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputSource (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

/*
  Two thresholds:
  1. Eucledian Distance between points in cluster
  2. Difference between their 'a' and 'b' values
*/
bool enforceLabSimilarity (const PointTypeColor& point_a, const PointTypeColor& point_b, float squared_distance)
{
  float dist = ((pow((point_a.x - point_b.x),2))+(pow((point_a.y - point_b.y),2))+(pow((point_a.z - point_b.z),2))) * 1000;
  //std::cout << "Distance: " << dist;
  if (dist < 0.5)
  {
    if ((abs (point_a.g - point_b.g) < 8.5f) && (abs (point_a.b - point_b.b) < 8.5f)) // compare a and b values (l=0) //7.5
      return (true);
  }
  return (false);
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

double objectAlignment(pcl::PointCloud<PointTypeColor>::Ptr scene, std::vector<FeatureCloud> object_templates)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < scene->points.size(); i++)
  {
    pcl::PointXYZ xyz(scene->points[i].x, scene->points[i].y, scene->points[i].z);
    cloud_xyz->push_back(xyz);
  }

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud_xyz);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size (); ++i)
  {
    template_align.addTemplateCloud (object_templates[i]);
  }
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  /*
  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  */

  return (best_alignment.fitness_score);
}


// Datatype to describe each element in pattern
struct Shape
{
  std::string name;
  pcl::PointXYZ centroid;
};

int main (int argc, char** argv)
{
  /*
    Note: The lowest block in patttern should be in at 0th index
    of the array and topmost block at the last element position.
  */
  Shape pattern [1];
  // Define pattern to be formed.
  pattern[0].name = "cylinder";
  pattern[0].centroid = pcl::PointXYZ::PointXYZ (0.1, 0.1, 0.1);

  // Data containers used
  pcl::PointCloud<PointTypeColor>::Ptr cloud_color_in (new pcl::PointCloud<PointTypeColor>),
                                      cloud_color_out (new pcl::PointCloud<PointTypeColor>);
  pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters),
                          small_clusters (new pcl::IndicesClusters),
                          large_clusters (new pcl::IndicesClusters);
  pcl::search::KdTree<PointTypeColor>::Ptr search_tree (new pcl::search::KdTree<PointTypeColor>);
  pcl::console::TicToc tt;

  // Load the input point cloud
  std::cerr << "Loading...\n", tt.tic ();
  pcl::io::loadPCDFile ("color_scene2.pcd", *cloud_color_in);
  std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_color_in->points.size () << " points\n";

  // Downsample the cloud using a Voxel Grid class
  std::cerr << "Downsampling...\n", tt.tic ();
  pcl::VoxelGrid<PointTypeColor> vg;
  vg.setInputCloud (cloud_color_in);
  vg.setLeafSize (0.005f, 0.005f, 0.005f);
  vg.setDownsampleAllData (true);
  vg.filter (*cloud_color_out);
  std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_color_out->points.size () << " points\n";

  // Convert rgb to lab values
  for (size_t i = 0; i < cloud_color_out->points.size (); ++i)
  {
    Eigen::Vector3i colorRGB = {cloud_color_out->points[i].r,
                                cloud_color_out->points[i].g,
                                cloud_color_out->points[i].b};

    Eigen::Vector3f lab = RGB2Lab (colorRGB);

    cloud_color_out->points[i].r = 0; //lab[0];
    cloud_color_out->points[i].g = lab[1];
    cloud_color_out->points[i].b = lab[2];
  }

  /* Clustering with "Normal similarity" criteria.

  // Set up a Normal Estimation class and merge data in cloud_with_normals
  std::cerr << "Computing normals...\n", tt.tic ();
  pcl::copyPointCloud (*cloud_color_out, *cloud_with_normals);
  pcl::NormalEstimation<PointTypeColor, PointTypeFull> ne;
  ne.setInputCloud (cloud_color_out);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch (1.0);
  ne.compute (*cloud_with_normals);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

  */

  // Set up a Conditional Euclidean Clustering class
  std::cerr << "Segmenting to clusters...\n", tt.tic ();
  pcl::ConditionalEuclideanClustering<PointTypeColor> cec (true);
  cec.setInputCloud (cloud_color_out); //cloud_with_normals);
  cec.setConditionFunction (&enforceLabSimilarity);
  cec.setClusterTolerance (10.0); //?
  cec.setMinClusterSize (30);
  cec.setMaxClusterSize (2000);
  cec.segment (*clusters);
  cec.getRemovedClusters (small_clusters, large_clusters);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
  std::cerr << "Number of clusters...\n" << clusters->size();

  /*
    Once clusters are obtained, read in elements from the
    pattern array and find coordinate of the each object in scene.
  */
  pcl::PointXYZ goal[sizeof (pattern)];
  for (int k = 0; k < sizeof (pattern); k++)
  {
    std::string object_file = pattern[k].name + ".txt";

    // Load the object templates specified in the object.txt file
    std::vector<FeatureCloud> object_templates;
    object_templates.resize (0);

    std::cerr << "File name..\n" << object_file;

    std::ifstream input_stream (object_file);
    while (input_stream.good ())
    {
      std::getline (input_stream, object_file);
      // Skip blank lines or comments
      if (object_file.empty () || object_file.at (0) == '#')
        continue;

      FeatureCloud template_cloud;
      template_cloud.loadInputCloud (object_file);
      object_templates.push_back (template_cloud);
      std::cerr << "total templates \n" << object_templates.size();
    }
    input_stream.close ();

    /*
      Get pointcloud for each cluster and find which cluster
      corresponds to the block that's specified in the pattern.
    */
    double min_score, score = 0.0;
    pcl::PointCloud<PointTypeColor>::Ptr cluster_cloud (new pcl::PointCloud<PointTypeColor>);
    pcl::PointCloud<PointTypeColor>::Ptr match_cloud (new pcl::PointCloud<PointTypeColor>);
    for (int i = 0; i < clusters->size (); ++i)
    {
      for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
      {
        cluster_cloud->push_back(cloud_color_out->points[(*clusters)[i].indices[j]]);
      }
      std::cerr << "before allign \n";
      score = objectAlignment(cluster_cloud, object_templates);
      if (i == 0)
      {
        min_score = score;
        pcl::copyPointCloud (*cluster_cloud, *match_cloud);
      }        
      if (score < min_score)
      {
        min_score = score;
        std::cerr << "min_score \n" << min_score;
        pcl::copyPointCloud (*cluster_cloud, *match_cloud);
      }
      cluster_cloud->clear();
    }
    pcl::visualization::PCLVisualizer *viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(match_cloud);
    viewer->addPointCloud<PointTypeColor>(match_cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->setBackgroundColor (0, 0, 0);
    viewer->spin();
    // goal[k] = centroid(cluster_cloud)
  }
  return (0);
}
