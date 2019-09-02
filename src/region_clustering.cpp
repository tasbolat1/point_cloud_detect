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

#define RED_THRESHOLD_R				180
#define RED_THRESHOLD_G				90
#define RED_THRESHOLD_B				90
#define RED_POINTS_SIZE_MAX			500

#define GREEN_THRESHOLD_R			100
#define GREEN_THRESHOLD_G			100
#define GREEN_THRESHOLD_B			100
#define GREEN_POINTS_SIZE_MAX		10000


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


	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud_downsampled);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	pass.filter (*indices);

	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud (cloud_downsampled);
	reg.setIndices (indices);
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (3);
	reg.setPointColorThreshold (4);
	reg.setRegionColorThreshold (10);
	reg.setMinClusterSize (80);

	std::vector <pcl::PointIndices> clusters;
  	reg.extract (clusters);

  	std::cout << "Number of total points: " << cloud_downsampled->points.size() << std::endl;
  	std::cout << "Number of clusters: " << clusters.size() << std::endl;

  	
  	// for(size_t i=0; i<clusters[0].indices.size(); i++) {
  	// 	pcl::PointXYZRGB new_point = cloud_downsampled->points[clusters[0].indices[i]];
  	// 	std::cout << int(new_point.r) << std::endl;
  	// }
  	//std::cout << cloud_downsampled->points[0] << std::endl;
  	std::vector<pcl::PointXYZRGB> red_candidate_locs;
  	std::vector<pcl::PointXYZRGB> green_candidate_locs;

  	pcl::PointXYZRGB xyzrgb_avg_p_selected;
  	pcl::PointXYZRGB xyzrgb_avg_p;
  	for (size_t i=0; i<clusters.size(); i++) {
  		 getAverageXYZRGBCluster(clusters[i], cloud_downsampled, &xyzrgb_avg_p);	


  		// RED MARKER SELECTION  
  		if ( (int(xyzrgb_avg_p.r) >= RED_THRESHOLD_R) && (int(xyzrgb_avg_p.g) <= RED_THRESHOLD_G) && (int(xyzrgb_avg_p.b) <= RED_THRESHOLD_B) ) {
  			if (clusters[i].indices.size() <= RED_POINTS_SIZE_MAX) {
  				red_candidate_locs.push_back(xyzrgb_avg_p);
  				//std::cout << int(xyzrgb_avg_p.r) << ", " << int(xyzrgb_avg_p.g) << ", " << int(xyzrgb_avg_p.b) << std::endl;
  				//std::cout <<  clusters[i].indices.size() << std::endl;
  			}
  		}

  		// GREEN MARKER SELECTION    && (int(xyzrgb_avg_p.r) <= GREEN_THRESHOLD_R) && (int(xyzrgb_avg_p.b) <= GREEN_THRESHOLD_B) 
  		if ( (int(xyzrgb_avg_p.g) >= GREEN_THRESHOLD_G)) {
  			if (clusters[i].indices.size() <= GREEN_POINTS_SIZE_MAX) {
  				green_candidate_locs.push_back(xyzrgb_avg_p);
  				std::cout << int(xyzrgb_avg_p.r) << ", " << int(xyzrgb_avg_p.g) << ", " << int(xyzrgb_avg_p.b) << std::endl;
  				std::cout <<  clusters[i].indices.size() << std::endl;
  			}
  		}

  	}

  	std::cout << green_candidate_locs.size() << std::endl;
  	

  	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

	// Visuzalize the cloud
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer = rgbVis(colored_cloud);
    for (size_t i=0; i<red_candidate_locs.size(); i++) {
    	viewer->addCoordinateSystem(0.1, red_candidate_locs[i].x, red_candidate_locs[i].y, red_candidate_locs[i].z);
    }
    for (size_t i=0; i<green_candidate_locs.size(); i++) {
    	viewer->addCoordinateSystem(0.1, green_candidate_locs[i].x, green_candidate_locs[i].y, green_candidate_locs[i].z);
    }
    while (!viewer->wasStopped ()) {
	viewer->spinOnce (100);
	std::this_thread::sleep_for(100ms);
	}

	return (0);
}