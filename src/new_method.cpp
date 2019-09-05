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
#include <pcl/segmentation/region_growing_rgb.h>
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



int main (int argc, char** argv)
{
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
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_downsampled_2 (new pcl::PointCloud<pcl::PointXYZHSV>);
	pass.filter (*cloud_downsampled_2);

  pcl::PassThrough<pcl::PointXYZHSV> pass2;
  pass2.setInputCloud (cloud_downsampled_2);
  pass2.setFilterFieldName ("s");
  pass2.setFilterLimits (0.4, 1);
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_downsampled_3 (new pcl::PointCloud<pcl::PointXYZHSV>);
  pass2.filter (*cloud_downsampled_3);

  pcl::PassThrough<pcl::PointXYZHSV> pass3;
  pass3.setInputCloud (cloud_downsampled_3);
  pass3.setFilterFieldName ("v");
  pass3.setFilterLimits (0, 1);
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_downsampled_4 (new pcl::PointCloud<pcl::PointXYZHSV>);
  pass3.filter (*cloud_downsampled_4);

  /*for(pcl::PointCloud<pcl::PointXYZHSV>::iterator it = cloud_downsampled_4->begin(); it!= cloud_downsampled_4->end(); it++)
  {
    std::cout << it->h << ", " << it->s << ", " << it->v << std::endl;
  }*/

  //Convert back to XYZRGB for visualization
  //PointCloudXYZHSVtoXYZRGB doesn't exist in PCL.
  //http://www.pcl-users.org/PCLVisualizer-is-showing-black-and-white-HSV-Point-Cloud-td4045564.html
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  PointCloudXYZHSVtoXYZRGB(*cloud_downsampled_2, *rgb_cloud);




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
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (5);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (rgb_cloud);
  ec.extract (cluster_indices);


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
  // for(size_t i=0; i<cloud_cluster.size(); i++) {
  //   viewer = rgbVis(cloud_cluster[i]);
  // }

  viewer = rgbVis(rgb_cloud);
  // viewer = rgbVis(cloud_cluster[2]);
  // viewer = rgbVis(cloud_cluster[3]);
  
  while (!viewer->wasStopped ()) {
  	viewer->spinOnce (100);
  	std::this_thread::sleep_for(100ms);
	}

	return (0);
}