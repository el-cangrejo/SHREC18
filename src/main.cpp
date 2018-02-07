#include <iostream>
#include "comparison.h"
#include "utils.h"

int main(int argc, char** argv) {
	std::string queryFilepath = "shrec18_recognition/Queries/5.ply";
	pcl::PointCloud<pcl::PointXYZ> queryCloud;
	pcl::PointCloud<pcl::Normal> queryNormals;
	pcl::PolygonMesh queryMesh;
	if (pcl::io::loadPLYFile(queryFilepath, queryMesh) == -1) {
		return -1;
	} else
		std::cout << "Query model : " << queryMesh.cloud.data.size()
			  << "\n";
	pcl::fromPCLPointCloud2(queryMesh.cloud, queryCloud);
	computeNormals(queryMesh, queryCloud, queryNormals);
	CloudWithNormals queryCloudWithNormals(queryCloud, queryNormals);

	float radius = 0.05;
	int index = 15000;
	std::cout << "Computing features.." << std::endl;
	auto queryFeatures =
	    computeFeatures_SHOT352(queryCloudWithNormals, radius);

	std::cout << "Computing distances.." << std::endl;
	std::vector<float> distances =
	    selfFeatureDistance(queryFeatures, index);

	pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;
	std::cout << "Creating RGB cloud.." << std::endl;
	createRGBCloud(queryCloud, distances, rgbCloud);

	auto& cloudToVisualize = rgbCloud;
	normalizeCloud(cloudToVisualize);
	// enterViewerLoop(queryMesh);
	enterViewerLoop(cloudToVisualize, rgbCloud[index], radius);

	// std::string targetModel = "shrec18_recognition/Dataset/1.ply";
	// pcl::PointCloud<pcl::PointXYZ> targetCloud;
	// pcl::PointCloud<pcl::Normal> targetNormals;
	// pcl::PolygonMesh targetMesh;
	// if (pcl::io::loadPLYFile(targetModel, targetMesh) == -1) {
	// return -1;
	// }
	// std::cout << "Target model : " << targetMesh.cloud.data.size() <<
	// "\n";
	//
	// auto queryFeatures =
	//    k
	//    computeFeatures_SHOT352(queryCloudWithNormals, searchQueryRadius);

	// pcl::fromPCLPointCloud2(queryMesh.cloud, targetCloud);
	// computeNormals(targetMesh, targetCloud, targetNormals);
	// CloudWithNormals targetCloudWithNormals(targetCloud, targetNormals);
}
