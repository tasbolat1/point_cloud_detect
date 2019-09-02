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


int main (int argc, char** argv)
{
	// Open one frame
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr pointCloud (new pcl::PointCloud <pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("../data/3d/colors_move_1567153295615528.pcd", *pointCloud) == -1 ) {
    	std::cout << "Cloud reading failed." << std::endl;
    	return (-1);
    }
    

	// Create the filtering object: downsample the dataset using a leaf size of 1mm
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (pointCloud);
	sor.setLeafSize (0.001f, 0.001f, 0.001f);
	sor.filter (*cloud_downsampled);



	// Visuzalize the cloud
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer = rgbVis(cloud_downsampled);
    while (!viewer->wasStopped ()) {
	viewer->spinOnce (100);
	std::this_thread::sleep_for(100ms);
	}

	return (0);
}