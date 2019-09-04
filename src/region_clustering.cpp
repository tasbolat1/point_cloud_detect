#include <iostream>
#include <thread>
#include <vector>

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


pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  return (viewer);
}


void PointCloudXYZHSVtoXYZRGB(const pcl::PointCloud<pcl::PointXYZHSV>& in, pcl::PointCloud<pcl::PointXYZRGB>& out)
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
  if (argc != 2){
    std::cout << "Wrong number of arguments!" << std::endl;
    return (-1);
  }
  // Open one frame
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr pointCloud (new pcl::PointCloud <pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("../data/3d/colors_move_1567153295615528.pcd", *pointCloud) == -1 ) {
      std::cout << "Cloud reading failed." << std::endl;
      return (-1);
    }

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud (new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::PointCloudXYZRGBtoXYZHSV(*pointCloud, *hsv_cloud);

  
  // Voxel grid filtering destroys HSV information!
  // Create the filtering object: downsample the dataset using a leaf size of 1mm
  // pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZHSV>);
  // pcl::VoxelGrid<pcl::PointXYZHSV> sor;
  // sor.setInputCloud (hsv_cloud);
  // sor.setLeafSize (0.001f, 0.001f, 0.001f);
  // sor.filter (*cloud_downsampled);

  //For multiple filters, I am lazy, https://stackoverflow.com/questions/45790828/remove-points-outside-defined-3d-box-inside-pcl-visualizer
  pcl::PassThrough<pcl::PointXYZHSV> pass;
  pass.setInputCloud (hsv_cloud);
  pass.setFilterFieldName ("h");
  pass.setFilterLimits (60, 360);
  pass.filter (*hsv_cloud);

  pass.setInputCloud (hsv_cloud);
  pass.setFilterFieldName ("s");
  pass.setFilterLimits (0.4, 1);
  pass.filter (*hsv_cloud);

  pass.setInputCloud (hsv_cloud);
  pass.setFilterFieldName ("v");
  pass.setFilterLimits (0, 1);
  pass.filter (*hsv_cloud);

  // for(pcl::PointCloud<pcl::PointXYZHSV>::iterator it = hsv_cloud->begin(); it!= hsv_cloud->end(); it++)
  // {
  //   std::cout << it->h << ", " << it->s << ", " << it->v << std::endl;
  // }

  //Convert back to XYZRGB for visualization
  //PointCloudXYZHSVtoXYZRGB doesn't exist in PCL.
  //http://www.pcl-users.org/PCLVisualizer-is-showing-black-and-white-HSV-Point-Cloud-td4045564.html
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  PointCloudXYZHSVtoXYZRGB(*hsv_cloud, *rgb_cloud);




  /*=============================================================================================
  * New parts by Jeffrey
  * 3 Sept 2019 1730hrs
  =============================================================================================*/
  // Voxel grid filtering to downsample rgb_cloud (Optional)
  // Create the filtering object: downsample the dataset using a leaf size of 1mm
  /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (rgb_cloud);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (*cloud_downsampled);
*/


  // Perform Euclidean Cluster Extraction to segment out the various clusters
  // Clusters will be stored in cluster_indices
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (rgb_cloud);

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
  //reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (3);
  reg.setPointColorThreshold (4);
  reg.setRegionColorThreshold (30);
  reg.setMinClusterSize (60);
  reg.extract (cluster_indices);


  pcl::PointXYZRGB xyzrgb_avg_p;
  for (size_t i=0; i<cluster_indices.size(); i++) {
    getAverageXYZRGBCluster(cluster_indices[i], rgb_cloud, &xyzrgb_avg_p); 
    std::cout << "Cluster " << i << " average RGB: " << int(xyzrgb_avg_p.r) << ", " << int(xyzrgb_avg_p.g) << ", " << int(xyzrgb_avg_p.b) << std::endl;
    std::cout << "Number of points: " << cluster_indices[i].indices.size() << std::endl;
  }


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

  std::cout << "No of clusters: " << cluster_indices.size() << std::endl;
  /*=============================================================================================*/





  // Visuzalize the cloud
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer = rgbVis(cloud_cluster[std::stod(argv[1])]);
  //viewer = rgbVis(rgb_cloud);
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  return (0);
}