#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/impl/point_types.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>

typedef pcl::PointXYZRGB PointTypeColor;
typedef pcl::PointXYZINormal PointTypeFull;

bool
enforceLabSimilarity (const PointTypeColor& point_a, const PointTypeColor& point_b, float squared_distance)
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

int
main (int argc, char** argv)
{
  // Data containers used
  pcl::PointCloud<PointTypeColor>::Ptr cloud_color_in (new pcl::PointCloud<PointTypeColor>), cloud_color_out (new pcl::PointCloud<PointTypeColor>);
  pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
  pcl::search::KdTree<PointTypeColor>::Ptr search_tree (new pcl::search::KdTree<PointTypeColor>);
  pcl::console::TicToc tt;

  // Load the input point cloud
  std::cerr << "Loading...\n", tt.tic ();
  pcl::io::loadPCDFile ("color_scene3.pcd", *cloud_color_in);
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

  // Set up a Normal Estimation class and merge data in cloud_with_normals
  std::cerr << "Computing normals...\n", tt.tic ();
  pcl::copyPointCloud (*cloud_color_out, *cloud_with_normals);
  pcl::NormalEstimation<PointTypeColor, PointTypeFull> ne;
  ne.setInputCloud (cloud_color_out);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch (1.0);
  ne.compute (*cloud_with_normals);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

  //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_act (new pcl::PointCloud<pcl::PointXYZI>);
  //pcl::PointCloudXYZRGBtoXYZI(*filCloud, *cloud_act);

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

  pcl::PointCloud<PointTypeColor>::Ptr cloud_final (new pcl::PointCloud<PointTypeColor>);

  // Visualization of the output

  for (int i = 0; i < small_clusters->size (); ++i)
    for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
    {
      cloud_color_out->points[(*small_clusters)[i].indices[j]].r = 0;
      cloud_color_out->points[(*small_clusters)[i].indices[j]].g = 0;
      cloud_color_out->points[(*small_clusters)[i].indices[j]].b = 0;
    }
  for (int i = 0; i < large_clusters->size (); ++i)
    for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
    {
      cloud_color_out->points[(*large_clusters)[i].indices[j]].r = 0;
      cloud_color_out->points[(*large_clusters)[i].indices[j]].g = 0;
      cloud_color_out->points[(*large_clusters)[i].indices[j]].b = 0;
      //cloud_act->push_back(cloud_out->points[(*large_clusters)[i].indices[j]]);
    }

  std::vector<unsigned char> colors;
  for (size_t i_segment = 0; i_segment < (clusters->size()); i_segment++)
  {
    colors.push_back (static_cast<unsigned char> (rand () % 256));
    colors.push_back (static_cast<unsigned char> (rand () % 256));
    colors.push_back (static_cast<unsigned char> (rand () % 256));
  }

  int next_color = 0;
  for (int i = 0; i < clusters->size (); ++i)
  {
    for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
    {
      cloud_color_out->points[(*clusters)[i].indices[j]].b = colors[3 * next_color];
      cloud_color_out->points[(*clusters)[i].indices[j]].g = colors[3 * next_color + 1];
      cloud_color_out->points[(*clusters)[i].indices[j]].r = colors[3 * next_color + 2];
      cloud_final->push_back(cloud_color_out->points[(*clusters)[i].indices[j]]);
    }
    next_color++;
  }

  pcl::visualization::PCLVisualizer *viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_final);
  viewer->addPointCloud<PointTypeColor>(cloud_final, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->setBackgroundColor (0, 0, 0);
  viewer->spin();

/*
  // Save the output point cloud
  std::cerr << "Saving...\n", tt.tic ();
  pcl::io::savePCDFile ("output.pcd", *cloud_act);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
*/

  return (0);
}
