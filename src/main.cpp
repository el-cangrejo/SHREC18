#include <iostream>
#include "utils.hpp"

int main(int argc, char **argv) {
	std::cout << "Hello SHREC2018!\n";

	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
	// pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(
	    new pcl::PointCloud<pcl::Normal>);
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

	// std::string queryModel = "shrec18_recognition/Queries/2.ply";
	// std::string dataModel = "shrec18_recognition/Dataset/2.ply";
	std::string queryModel = "1.ply";
	if (pcl::io::loadPLYFile(queryModel, *mesh) == -1) {
		return -1;
	}

	std::cout << "Loaded " << mesh->cloud.data.size() << " data points"
		  << std::endl;

	// normalizeCloud(cloud);

	// auto cloud_normals = execute_normalEstimation(cloud);
	// std::cout << "Computed " << cloud_normals->points.size() << "
	// normals" << std::endl;

	// auto fpfhs = execute_featureEstimation(cloud, cloud_normals);
	// std::cout << "Computed " << fpfhs->points.size() << " features" <<
	// std::endl;

	enterViewerLoop(cloud, cloud_normals);
}
