#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>  
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <pcl/filters/voxel_grid.h>
int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	clock_t start, end;
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	const float leaf = 0.005f;
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (cloud);
	grid.filter (*cloud);

	std::cout<<cloud->points.size()<<std::endl;
	start = clock();
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.01);

	normalEstimation.compute(*normals);
	end = clock();
	std::cout<<normals->points.size()<<std::endl;
	std::cout<<"estimate the normals:"<<(end - start)/CLOCKS_PER_SEC<<"s"<<std::endl;
	start = clock();

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation1;
	normalEstimation1.setRadiusSearch(0.01);
	normalEstimation1.setNumberOfThreads(12);

	normalEstimation1.setInputCloud(cloud);
	normalEstimation1.compute(*normals);
 
	end = clock();
	std::cout<<normals->points.size()<<std::endl;
	std::cout<<"estimate the normalsOPM:"<<(end - start)/CLOCKS_PER_SEC<<"s"<<std::endl;
	// Visualize them.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	// Display one normal out of 20, as a line of length 3cm.
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 20, 0.03, "normals");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
