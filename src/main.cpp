#include <iostream>
#include <tuple>
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

	//std::string queryModel = "shrec18_recognition/Dataset/" +
	//			std::to_string(atoi(argv[1])) + ".ply";
	//
	std::string queryModel = "shrec18_recognition/Queries/" +
				std::to_string(atoi(argv[1])) + ".ply";
	pcl::PointCloud<pcl::PointXYZ> cloud_q;
	pcl::PointCloud<pcl::Normal> normals_q;
	pcl::PolygonMesh mesh_q;

	std::string targetModel = "shrec18_recognition/Dataset/" +
				std::to_string(atoi(argv[7])) + ".ply";
	pcl::PointCloud<pcl::PointXYZ> cloud_t;
	pcl::PointCloud<pcl::Normal> normals_t;
	pcl::PolygonMesh mesh_t;

	
	if (pcl::io::loadPLYFile(queryModel, mesh_q) == -1) {
		return -1;
	}
	if (pcl::io::loadPLYFile(targetModel, mesh_t) == -1) {
		return -1;
	}

	int idx = atoi(argv[2]);
	float inner_radius = atof(argv[3]);
	float outer_radius = atof(argv[4]);

	pcl::fromPCLPointCloud2(mesh_q.cloud, cloud_q);
	std::cout << "Query model : " << cloud_q.points.size() << "\n";

	computeNormals(mesh_q, cloud_q, normals_q);
	CloudWithNormals cloud_qWithNormals(cloud_q, normals_q);
	std::cout << "Computing query features\n";
	auto features_q = computeFeaturesSHOT352(cloud_qWithNormals, inner_radius);

	pcl::fromPCLPointCloud2(mesh_t.cloud, cloud_t);
	std::cout << "Target model : " << cloud_t.points.size() << "\n";

	computeNormals(mesh_t, cloud_t, normals_t);
	CloudWithNormals cloud_tWithNormals(cloud_t, normals_t);
	std::cout << "Computing target features\n";
	auto features_t = computeFeaturesSHOT352(cloud_tWithNormals, inner_radius);

	std::cout << "Computing self distances for query\n";
	std::vector<float> self_distances = selfFeatureDistance(features_q, idx);
	thresholdVector(self_distances, atof(argv[6]));


	std::vector<int> nodes_idx{idx};

	int N = atoi(argv[5]);
	std::vector<int> indices_vec;
	std::vector<int> checking_idxs;

	for (int i = 0; i < N; i++) {
		indices_vec = findIndices(cloud_q, inner_radius, outer_radius, nodes_idx, self_distances);
		std::cout << "Found " << indices_vec.size()  << " in rep " << i << std::endl;
		if (indices_vec.size() == 0) break;

		auto features_vec = matchIndicesFeatures(indices_vec, features_q);
		
		int closestFeature_idx = findClosestFeature(features_vec, features_q[nodes_idx.back()]);
		
		nodes_idx.push_back(indices_vec[closestFeature_idx]);

		checking_idxs.insert(checking_idxs.end(), indices_vec.begin(), indices_vec.end());
	}

	for (auto i : nodes_idx)
		std::cout << "Index of closest feature: " << i  << std::endl;

	pcl::SHOT352 mean_feature = computeMeanFeature(features_q, nodes_idx);

	for (int j = 0; j < 352; ++j) {
		std::cout << mean_feature.descriptor[j] << "\n";
	}

	std::cout << "Computing distances to target\n";
	std::vector<float> target_distances = 
		//computeFeatureDistancesFromTargetModel<pcl::SHOT352>(features_t, features_q.points[idx]);
		computeFeatureDistancesFromTargetModel<pcl::SHOT352>(features_t, mean_feature);
	//thresholdVector(target_distances, atof(argv[6]));
		
	std::vector<std::tuple<int, float>> dist_idx;
	for (int i = 0; i < target_distances.size(); ++i) {
		dist_idx.push_back(std::make_tuple(i, target_distances[i]));
	}

	std::sort(std::begin(dist_idx), std::end(dist_idx), [](auto const &t1, auto const &t2) {
					return std::get<1>(t1) < std::get<1>(t2); // or use a custom compare function
	});
	//auto min_dist_point = std::min_element(target_distances.begin(), target_distances.end());
	//auto min_dist_point = std::min_element(target_distances.begin(), target_distances.end());

	//std::vector<int> min_point{min_dist_point - target_distances.begin()};
	std::vector<int> min_point;
	for (int i = 0; i < 5; ++i) {
		min_point.push_back(std::get<0>(dist_idx[i]));
	}

	pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
	createRGBCloud(cloud_t, target_distances, cloud_rgb);
	centerCloud<pcl::PointXYZRGB>(cloud_rgb);

	enterViewerLoop(cloud_rgb, normals_t, min_point, .1);
}

