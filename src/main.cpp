#include <iostream>
#include <utils.hpp>

int main(int argc, char **argv) {
	std::cout << "Hello SHREC2018!\n";

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::Normal> normals;
	pcl::PolygonMesh mesh;

	std::string queryModel = "shrec18_recognition/Queries/2.ply";
	// std::string dataModel = "shrec18_recognition/Dataset/1.ply";

	if (pcl::io::loadPLYFile(queryModel, mesh) == -1) {
		return -1;
	}

	std::cout << "Loaded query mesh with: " << mesh.cloud.data.size()
		  << " data points" << std::endl;

	pcl::fromPCLPointCloud2(mesh.cloud, cloud);
	//normalizeCloud(cloud);
	computeNormals(mesh, cloud, normals);

	pcl::PointCloud<pcl::PFHSignature125> features;
	computeFeatures(cloud, normals, features);
	std::vector<float> distances;
	computeFeatureDistancesFromTarget(features, 0, distances);
	pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;
	createRGBCloud(cloud, distances, rgbCloud);
	enterViewerLoopMesh(mesh, rgbCloud, normals);
}
