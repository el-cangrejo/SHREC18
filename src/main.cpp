#include <iostream>
#include <utils.hpp>
#include "comparison.h"

int main(int argc, char **argv) {
	std::cout << "Hello SHREC2018!\n";

	std::string queryModel = "shrec18_recognition/Dataset/3.ply";
	//std::string queryModel = "shrec18_recognition/Queries/2.ply";
	pcl::PointCloud<pcl::PointXYZ> queryCloud;
	pcl::PointCloud<pcl::Normal> queryNormals;
	pcl::PolygonMesh queryMesh;

	//std::string targetModel = "shrec18_recognition/Dataset/6.ply";
	//pcl::PointCloud<pcl::PointXYZ> targetCloud;
	//pcl::PointCloud<pcl::Normal> targetNormals;
	//pcl::PolygonMesh targetMesh;

	//// comparison parameters
	//float searchQueryRadius = 1.5;
	//float searchTargetRadius = 0.5;
	//int targetPointIndex = 2000;

	if (pcl::io::loadPLYFile(queryModel, queryMesh) == -1) {
		return -1;
	}
	//if (pcl::io::loadPLYFile(targetModel, targetMesh) == -1) {
	//	return -1;
	//}

	std::cout << "Query model : " << queryMesh.cloud.data.size() << "\n";
	//std::cout << "Target model : " << targetMesh.cloud.data.size() << "\n";
	//
	pcl::fromPCLPointCloud2(queryMesh.cloud, queryCloud);
	computeNormals(queryMesh, queryCloud, queryNormals);
	CloudWithNormals queryCloudWithNormals(queryCloud, queryNormals);
	//auto queryFeatures =
	//    computeFeatures_SHOT352(queryCloudWithNormals, searchQueryRadius);

	//pcl::fromPCLPointCloud2(targetMesh.cloud, targetCloud);
	//computeNormals(targetMesh, targetCloud, targetNormals);
	//CloudWithNormals targetCloudWithNormals(targetCloud, targetNormals);
	//auto targetFeatures =
	//    computeFeatures_SHOT352(targetCloudWithNormals, searchTargetRadius);

	//int featureSize = 352;
	//std::vector<float> distances = computeFeatureDistancesFromTargetModel(
	//    targetFeatures, queryFeatures[targetPointIndex], featureSize);
	//pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;
	//createRGBCloud(targetCloud, distances, rgbCloud);

	int idx = 0;
	float inner_radius = 0.5;
	float outer_radius = 1.5;
	auto pfh_inner = computePointFeatures_PFH(queryCloudWithNormals, idx, 20);
	auto pfh_outer = computePointFeatures_PFH(queryCloudWithNormals, idx, 40);

	auto h = histogramDifference(pfh_inner.points[idx].histogram, pfh_outer.points[idx].histogram, 125);

	for (auto i : h)
		std::cout << i << "\n";

	enterViewerLoop(queryCloud, queryNormals, queryCloud.points[idx], inner_radius);
}
