#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/impl/search.hpp>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE

pcl::PointCloud<pcl::Normal>::Ptr	execute_normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (cloud);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		ne.setSearchMethod (tree);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		ne.setRadiusSearch (0.03);
		ne.compute (*cloud_normals);
	
		return cloud_normals;
	}

#include <pcl/features/fpfh.h>
template<typename Cloud,typename CloudNormals>
size_t execute_featureEstimation(Cloud cloud,CloudNormals normals){
	  // Create the FPFH estimation class, and pass the input dataset+normals to
		// it
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	  fpfh.setInputCloud (cloud);
		  fpfh.setInputNormals (normals);

			// Create an empty kdtree representation, and pass it to the FPFH
			// estimation object.
			//Its content will be filled inside the object, based on the given
			//   input dataset (as no other search surface is given).
			  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

				  fpfh.setSearchMethod (tree);

					  // Output datasets
					  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

						// Use all neighbors in a sphere of radius 5cm
						//   // IMPORTANT: the radius used here has to be larger than the
						//   radius used to estimate the surface normals!!!
						  fpfh.setRadiusSearch (0.05);

							  // Compute the features
								  fpfh.compute (*fpfhs);

									return fpfhs->points.size ();
}

template<typename Cloud,typename CloudNormals>
void enterViewerLoop(Cloud cloud,CloudNormals normals){
	pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
	//viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");
	viewer.setBackgroundColor (0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud,normals, 10, 0.05, "normals");
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();
	while (!viewer.wasStopped ()) {
		viewer.spinOnce (100);
	}

}
int main (int argc, char** argv) {
	std::cout << "Hello SHREC2018!\n";

	 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	 std::string queryModel="shrec18_recognition/Queries/2.ply";
	 std::string dataModel="shrec18_recognition/Dataset/2.ply";
  if (pcl::io::loadPLYFile<pcl::PointXYZ> (queryModel, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << cloud->points.size() << " data points" << std::endl; 

	auto cloud_normals=execute_normalEstimation(cloud);
  std::cout << "Computed " << cloud->points.size() << " normals" << std::endl;

	auto featuresSize=execute_featureEstimation(cloud,cloud_normals);
	std::cout<<"Computed "<<featuresSize<<" features"<<std::endl;

	enterViewerLoop(cloud,cloud_normals);
}
