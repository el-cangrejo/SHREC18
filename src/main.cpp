#include <iostream>
#include <utils.hpp>

int main(int argc, char **argv) {
	std::cout << "Hello SHREC2018!\n";

	std::string queryModel = "shrec18_recognition/Queries/3.ply";
	pcl::PointCloud<pcl::PointXYZ> queryCloud;
	pcl::PointCloud<pcl::Normal> queryNormals;
	pcl::PolygonMesh queryMesh;
	pcl::PointCloud<pcl::FPFHSignature33> queryFeatures;

	std::string targetModel = "shrec18_recognition/Dataset/1.ply";
	pcl::PointCloud<pcl::PointXYZ> targetCloud;
	pcl::PointCloud<pcl::Normal> targetNormals;
	pcl::PolygonMesh targetMesh;
	pcl::PointCloud<pcl::FPFHSignature33> targetFeatures;


	float searchRadius = 0.12;
	int targetPointIndex = 2000;
	std::vector<float> distances;

	if (pcl::io::loadPLYFile(queryModel, queryMesh) == -1) {
		return -1;
	}
	if (pcl::io::loadPLYFile(targetModel, targetMesh) == -1) {
		return -1;
	}

	pcl::fromPCLPointCloud2(queryMesh.cloud, queryCloud);
	computeNormals(queryMesh, queryCloud, queryNormals);
	computeFeatures(queryCloud, queryNormals, queryFeatures, searchRadius);

	pcl::fromPCLPointCloud2(targetMesh.cloud, targetCloud);
	computeNormals(targetMesh, targetCloud, targetNormals);
	computeFeatures(targetCloud, targetNormals, targetFeatures, searchRadius);


	computeFeatureDistancesFromTargetModel<pcl::FPFHSignature33>(targetFeatures, queryFeatures[targetPointIndex], distances);
	pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;
	createRGBCloud(targetCloud, distances, rgbCloud);

	enterViewerLoop(rgbCloud, targetNormals, targetCloud.points[0], searchRadius);
}
