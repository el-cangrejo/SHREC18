#include <iostream>
#include <string>
#include "comparison.h"
#include "helpers.hpp"
#include "viewer.hpp"
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/principal_curvatures.h>

int main(int argc, char **argv) {
	std::cout << "Hello SHREC2018!\n";

	std::string queryModel = "shrec18_recognition/Dataset/" +
				std::to_string(atoi(argv[1])) + ".ply";
	//std::string queryModel = "shrec18_recognition/Queries/" +
	//			 std::to_string(atoi(argv[1])) + ".ply";
	pcl::PointCloud<pcl::PointXYZ> cloud_q;
	pcl::PointCloud<pcl::Normal> normals_q;
	pcl::PolygonMesh mesh_q;

	// std::string targetModel = "shrec18_recognition/Dataset/6.ply";
	// pcl::PointCloud<pcl::PointXYZ> targetCloud;
	// pcl::PointCloud<pcl::Normal> targetNormals;
	// pcl::PolygonMesh targetMesh;

	if (pcl::io::loadPLYFile(queryModel, mesh_q) == -1) {
		return -1;
	}
	// if (pcl::io::loadPLYFile(targetModel, targetMesh) == -1) {
	//	return -1;
	//}

	int idx = atoi(argv[2]);
	float inner_radius = atof(argv[3]);
	float outer_radius = atof(argv[4]);

	pcl::fromPCLPointCloud2(mesh_q.cloud, cloud_q);

	std::cout << "Query model : " << cloud_q.points.size() << "\n";

	computeNormals(mesh_q, cloud_q, normals_q);

	CloudWithNormals cloud_qWithNormals(cloud_q, normals_q);
	auto features_q = computeFeaturesSHOT352(cloud_qWithNormals, inner_radius);

	// pcl::fromPCLPointCloud2(targetMesh.cloud, targetCloud);
	// computeNormals(targetMesh, targetCloud, targetNormals);
	// CloudWithNormals targetCloudWithNormals(targetCloud, targetNormals);
	// auto targetFeatures =
	//    computeFeatures_SHOT352(targetCloudWithNormals,
	//    searchTargetRadius);

	std::vector<float> distances = selfFeatureDistance(features_q, idx);
	auto result = minmax_element(distances.begin(), distances.end());
	float mean = atof(argv[6]) * (distances[result.first - distances.begin()] + distances[result.second - distances.begin()]);
	for (int i = 0; i < distances.size(); ++i) {
		if (distances[i] < mean) distances[i] = 0;
		else distances[i] = 255;	
	}

	pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
	createRGBCloud(cloud_q, distances, cloud_rgb);
	centerCloud<pcl::PointXYZRGB>(cloud_rgb);
	std::vector<int> nodes_idx{idx};

	int N = atoi(argv[5]);
	std::vector<int> indices_vec;
	std::vector<int> checking_idxs;

	for (int i = 0; i < N; i++) {
		indices_vec = findIndices(cloud_q, inner_radius, outer_radius, nodes_idx, distances);
		std::cout << "Found " << indices_vec.size()  << " in rep " << i << std::endl;
		if (indices_vec.size() == 0) break;

		auto features_vec = matchIndicesFeatures(indices_vec, features_q);
		
		int closestFeature_idx = findClosestFeature(features_vec, features_q[nodes_idx.back()]);
		
		nodes_idx.push_back(indices_vec[closestFeature_idx]);

		checking_idxs.insert(checking_idxs.end(), indices_vec.begin(), indices_vec.end());
	}

	for (auto i : nodes_idx)
		std::cout << "Index of closest feature: " << i  << std::endl;

	enterViewerLoop(cloud_rgb, normals_q, nodes_idx, 0.05);
}
