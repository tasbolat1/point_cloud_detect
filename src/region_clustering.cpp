#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <tuple>
#include <math.h>     

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types_conversion.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>


using namespace std::chrono_literals;


pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::visualization::PCLVisualizer::Ptr viewer)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->removeAllPointClouds();
  viewer->setBackgroundColor (0,0,0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  return (viewer);
}

std::tuple<int, int> compute_color(int r, int g, int b){
  int n_joints = 6;
  int predefined_joint_colors[n_joints][3] = {{120,50,120}, {120,120,50}, {50,50,120}, {50,120,50}, {120,50,50}, {50,120,120}};
  std::vector<int> distances = {0, 0, 0 ,0 ,0, 0};
  for (size_t i = 0; i < n_joints; i++)
  {
    distances[i] = pow(r - predefined_joint_colors[i][0], 2) + pow(g - predefined_joint_colors[i][1], 2) + pow(b - predefined_joint_colors[i][2], 2);
  }
  int idx = std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));
  return std::make_tuple(idx, distances[idx]);
}


void PointCloudXYZHSVtoXYZRGB(pcl::PointCloud<pcl::PointXYZHSV>& in, pcl::PointCloud<pcl::PointXYZRGB>& out)
{
    out.width = in.width;
    out.height = in.height;
    for (size_t i = 0; i < in.points.size(); i++)
    {
        pcl::PointXYZRGB p;
        pcl::PointXYZHSVtoXYZRGB(in.points[i], p);
        out.points.push_back(p);
    }
} 

void getAverageXYZRGBCluster(pcl::PointIndices &cluster, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB *p) {
  // This function calculates average rgb color within a cluster
  std::vector<float> x(cluster.indices.size());
  std::vector<float> y(cluster.indices.size());
  std::vector<float> z(cluster.indices.size());

  std::vector<int> r(cluster.indices.size());
  std::vector<int> g(cluster.indices.size());
  std::vector<int> b(cluster.indices.size());


  for(size_t i=0; i<cluster.indices.size(); i++) {
      pcl::PointXYZRGB new_point = cloud->points[cluster.indices[i]];
      x[i] = new_point.x;
      y[i] = new_point.y;
      z[i] = new_point.z;
      r[i] = int(new_point.r);
      g[i] = int(new_point.g);
      b[i] = int(new_point.b);
    }

    //pcl::PointXYZRGB p;
    uint8_t r_, g_, b_;

    r_ = int( accumulate( r.begin(), r.end(), 0.0)/r.size() );
    g_ = int( accumulate( g.begin(), g.end(), 0.0)/g.size() );
    b_ = int( accumulate( b.begin(), b.end(), 0.0)/b.size() );


    float x_, y_, z_;
    x_ = accumulate( x.begin(), x.end(), 0.0)/x.size();
    y_ = accumulate( y.begin(), y.end(), 0.0)/y.size();
    z_ = accumulate( z.begin(), z.end(), 0.0)/z.size();

    uint32_t rgb = ((uint32_t)r_ << 16 | (uint32_t)g_ << 8 | (uint32_t)b_);

    p->rgb =  *reinterpret_cast<float*>(&rgb);
    p->x = x_;
    p->y = y_;
    p->z = z_;
    //return p;
}

int main (int argc, char** argv)
{
  if (argc != 1){
    std::cout << "Wrong number of arguments!" << std::endl;
    return (-1);
  }
  // Open one frame
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr pointCloud (new pcl::PointCloud <pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("../data/open.pcd", *pointCloud) == -1 ) {
      std::cout << "Cloud reading failed." << std::endl;
      return (-1);
    }

  auto start = std::chrono::high_resolution_clock::now();

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud (new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::PointCloudXYZRGBtoXYZHSV(*pointCloud, *hsv_cloud);

  //For multiple filters, I am lazy, https://stackoverflow.com/questions/45790828/remove-points-outside-defined-3d-box-inside-pcl-visualizer
  pcl::PassThrough<pcl::PointXYZHSV> pass;
  pass.setInputCloud (hsv_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 0.5);
  pass.filter (*hsv_cloud);

  pass.setInputCloud (hsv_cloud);
  pass.setFilterFieldName ("h");
  pass.setFilterLimits (0, 360);
  pass.filter (*hsv_cloud);

  pass.setInputCloud (hsv_cloud);
  pass.setFilterFieldName ("s");
  pass.setFilterLimits (0.4, 1);
  pass.filter (*hsv_cloud);

  pass.setInputCloud (hsv_cloud);
  pass.setFilterFieldName ("v");
  pass.setFilterLimits (0.2, 1);
  pass.filter (*hsv_cloud);

  // for(pcl::PointCloud<pcl::PointXYZHSV>::iterator it = hsv_cloud->begin(); it!= hsv_cloud->end(); it++)
  // {
  //   std::cout << it->h << ", " << it->s << ", " << it->v << std::endl;
  // }

  //Convert back to XYZRGB for visualization
  //PointCloudXYZHSVtoXYZRGB doesn't exist in PCL.
  //http://www.pcl-users.org/PCLVisualizer-is-showing-black-and-white-HSV-Point-Cloud-td4045564.html
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud_full (new pcl::PointCloud<pcl::PointXYZRGB>);
  PointCloudXYZHSVtoXYZRGB(*hsv_cloud, *rgb_cloud_full);




  /*=============================================================================================
  * New parts by Jeffrey
  * 3 Sept 2019 1730hrs
  =============================================================================================*/
  // Voxel grid filtering to downsample rgb_cloud (Optional)
  // Create the filtering object: downsample the dataset using a leaf size of 1mm
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (rgb_cloud_full);
  vg.setLeafSize (0.002f, 0.002f, 0.002f);
  vg.filter (*rgb_cloud);


  // Perform Euclidean Cluster Extraction to segment out the various clusters
  // Clusters will be stored in cluster_indices
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  std::vector<pcl::PointIndices> cluster_indices;
  // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // ec.setClusterTolerance (0.02); // 2cm
  // ec.setMinClusterSize (5);
  // ec.setMaxClusterSize (25000);
  // ec.setSearchMethod (tree);
  // ec.setInputCloud (rgb_cloud);
  // ec.extract (cluster_indices);
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (rgb_cloud);
  reg.setSearchMethod (tree);
  reg.setPointColorThreshold (10); // We grow regions by color. Should be not too small so that darker shades can be pulled into cluster. 
  reg.setRegionColorThreshold (9999); // We merge regions by position. 
  reg.setDistanceThreshold (0.0025); // A number that is slightly above voxel grid leaf size, so that only adjacent poins will be considered as neighbours. Too big means clusters will be pulled in indiscriminately.
  reg.setMinClusterSize (20); // A small number will do.
  reg.extract (cluster_indices);

  // Huge assumption; there is no way an actual marker will be assigned to some color that it does not belong to.
  pcl::PointXYZRGB xyzrgb_avg_p;
  int joint[6] = {-1, -1, -1, -1, -1, -1};
  int squared_distance[6] = {999999, 999999, 999999, 999999, 999999, 999999};
  double joint_coords[6][3] = {{-10,-10,-10}, {-10,-10,-10}, {-10,-10,-10}, {-10,-10,-10}, {-10,-10,-10}, {-10,-10,-10}};
  std::string color[6] = {"purple", "yellow", "blue", "green", "red", "cyan"};
  for (size_t i=0; i<cluster_indices.size(); i++) {
    getAverageXYZRGBCluster(cluster_indices[i], rgb_cloud, &xyzrgb_avg_p); 
    std::cout << "Cluster " << i << " average RGB: " << int(xyzrgb_avg_p.r) << ", " << int(xyzrgb_avg_p.g) << ", " << int(xyzrgb_avg_p.b) << std::endl;
    std::cout << "Cluster " << i << " average XYZ: " << xyzrgb_avg_p.x << ", " << xyzrgb_avg_p.y << ", " << xyzrgb_avg_p.z << std::endl;
    std::cout << "Number of points: " << cluster_indices[i].indices.size() << std::endl;
    std::tuple<int, int> joint_idx_weight = compute_color(int(xyzrgb_avg_p.r), int(xyzrgb_avg_p.g), int(xyzrgb_avg_p.b));
    std::cout << std::get<0>(joint_idx_weight) << ", " << std::get<1>(joint_idx_weight) << std::endl;
    if (joint[std::get<0>(joint_idx_weight)] == -1){
      joint[std::get<0>(joint_idx_weight)] = int(i);
      squared_distance[std::get<0>(joint_idx_weight)] = std::get<1>(joint_idx_weight);
      joint_coords[std::get<0>(joint_idx_weight)][0] = xyzrgb_avg_p.x;
      joint_coords[std::get<0>(joint_idx_weight)][1] = xyzrgb_avg_p.y;
      joint_coords[std::get<0>(joint_idx_weight)][2] = xyzrgb_avg_p.z;
    }
    else{
      if(std::get<1>(joint_idx_weight) < squared_distance[std::get<0>(joint_idx_weight)]){
        joint[std::get<0>(joint_idx_weight)] = int(i);
        squared_distance[std::get<0>(joint_idx_weight)] = std::get<1>(joint_idx_weight);
        joint_coords[std::get<0>(joint_idx_weight)][0] = xyzrgb_avg_p.x;
        joint_coords[std::get<0>(joint_idx_weight)][1] = xyzrgb_avg_p.y;
        joint_coords[std::get<0>(joint_idx_weight)][2] = xyzrgb_avg_p.z;
      }
    }
  }
  for (size_t i=0; i<6; i++){
    std::cout << color[i] << ": Cluster " << joint[i] << ", Squared distance: " << squared_distance[i] << std::endl;
  }
  for (size_t i=0; i<6; i++){
    std::cout << color[i] << " position: " << joint_coords[i][0] << "," << joint_coords[i][1] << "," << joint_coords[i][2] << std::endl;
  }

  auto stop = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> fp_ms = stop - start;
  std::cout << "Milliseconds taken to load PCD all the way to getting clusters and associated RGB centers: " << fp_ms.count() << std::endl; 


  // Convert all the clusters into XYZRGB for visualization
  // cloud_cluster vector contains each cluster's point cloud
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_cluster;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cluster->points.push_back (rgb_cloud->points[*pit]); //*
      cluster->width = cluster->points.size ();
      cluster->height = 1;
      cluster->is_dense = true;
      cloud_cluster.push_back(cluster);
      j++;

      //std::cout << "Cluster " << j << " size: " << cloud_cluster->points.size() << std::endl;
  }

  // Visuzalize the cloud
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer = rgbVis(rgb_cloud);
  int cluster_idx = 0;
  while (true) {
    rgbVis(cloud_cluster[cluster_idx % cluster_indices.size()], viewer);
    std::cout << "Displaying cluster " << cluster_idx % cluster_indices.size() << std::endl;
    viewer->spinOnce (3000);
    std::this_thread::sleep_for(3000ms);
    cluster_idx++;
  }

  return (0);
}
