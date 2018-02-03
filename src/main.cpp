#include <iostream>
#include <utils.hpp>
#include "comparison.h"

int main(int argc, char **argv) {
	std::cout << "Hello SHREC2018!\n";

	std::string queryModel = "shrec18_recognition/Queries/3.ply";
	pcl::PointCloud<pcl::PointXYZ> queryCloud;
	pcl::PointCloud<pcl::Normal> queryNormals;
	pcl::PolygonMesh queryMesh;

	std::string targetModel = "shrec18_recognition/Dataset/1.ply";
	pcl::PointCloud<pcl::PointXYZ> targetCloud;
	pcl::PointCloud<pcl::Normal> targetNormals;
	pcl::PolygonMesh targetMesh;

	// comparison parameters
	float searchRadius = 0.12;
	int targetPointIndex = 2000;

	if (pcl::io::loadPLYFile(queryModel, queryMesh) == -1) {
		return -1;
	}
	if (pcl::io::loadPLYFile(targetModel, targetMesh) == -1) {
		return -1;
	}

	pcl::fromPCLPointCloud2(queryMesh.cloud, queryCloud);
	computeNormals(queryMesh, queryCloud, queryNormals);
	CloudWithNormals queryCloudWithNormals(queryCloud, queryNormals);
	auto queryFeatures =
	    computeFeatures_PFH(queryCloudWithNormals, searchRadius);

	pcl::fromPCLPointCloud2(targetMesh.cloud, targetCloud);
	computeNormals(targetMesh, targetCloud, targetNormals);
	CloudWithNormals targetCloudWithNormals(targetCloud, targetNormals);
	auto targetFeatures =
	    computeFeatures_PFH(targetCloudWithNormals, searchRadius);

	int featureSize = 125;
	std::vector<float> distances = computeFeatureDistancesFromTargetModel(
	    targetFeatures, queryFeatures[targetPointIndex], featureSize);
	pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;
	createRGBCloud(targetCloud, distances, rgbCloud);

	enterViewerLoop(rgbCloud, targetNormals, targetCloud.points[0],
			searchRadius);
}
