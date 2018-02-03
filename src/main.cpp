#include <iostream>
#include <utils.hpp>
#include "comparison.h"

int main(int argc, char **argv) {
	std::cout << "Hello SHREC2018!\n";

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::Normal> normals;
	pcl::PolygonMesh mesh;

	std::string queryModel = "shrec18_recognition/Queries/5.ply";
	// std::string dataModel = "shrec18_recognition/Dataset/1.ply";

	if (pcl::io::loadPLYFile(queryModel, mesh) == -1) {
		return -1;
	}

	std::cout << "Loaded query mesh with: " << mesh.cloud.data.size()
		  << " data points" << std::endl;

	pcl::fromPCLPointCloud2(mesh.cloud, cloud);
	// normalizeCloud(cloud);
	computeNormals(mesh, cloud, normals);

	float searchRadius = 0.12;
	CloudWithNormals cloudWithNormals(cloud, normals);
	auto features = computeFeatures_PFH(cloudWithNormals, searchRadius);
	int targetPointIndex = 3000;
	pcl::PointXYZ target = cloud[targetPointIndex];
	std::vector<float> distances =
	    selfFeatureDistance<pcl::PFHSignature125>(features,
						      targetPointIndex);
	pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;
	createRGBCloud(cloud, distances, rgbCloud);
	//	enterViewerLoopMesh(mesh, rgbCloud, normals);
	enterViewerLoop(rgbCloud, normals, target, searchRadius);
}
