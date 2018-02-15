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

	int N = atoi(argv[5]);
	std::vector<int> query_graph = createGraph(cloud_q, features_q, 0.06, 
			0.15, self_distances, idx);
	
	pcl::SHOT352 mean_feature = computeMeanFeature(features_q, query_graph);

	std::cout << "Computing distances to target\n";
	std::vector<float> target_distances = 
		computeFeatureDistancesFromTargetModel<pcl::SHOT352>(features_t, features_q.points[idx]);
		//computeFeatureDistancesFromTargetModel<pcl::SHOT352>(features_t, mean_feature);
	thresholdVector(target_distances, atof(argv[6]));
	//thresholdVector(target_distances, atof(argv[6]));
		

	std::vector<int> target_graph;

	std::vector<std::tuple<int, float>> dist_idx;
	for (int i = 0; i < target_distances.size(); ++i) {
		dist_idx.push_back(std::make_tuple(i, target_distances[i]));
	}

	std::sort(std::begin(dist_idx), std::end(dist_idx), [](auto const &t1, auto const &t2) {
					return std::get<1>(t1) < std::get<1>(t2); // or use a custom compare function
	});


	for (int i = 0; i < atoi(argv[8]); ++i) {
		int min_point = findPointsWithMinDist(dist_idx, target_graph, cloud_t);

		std::cout << "Creating graph " << i << std::endl;
		std::vector<int> temp_nodes = createGraph(cloud_t, features_t, inner_radius, 
				outer_radius, target_distances, min_point);

		target_graph.insert(target_graph.end(), temp_nodes.begin(), temp_nodes.end());
	}

	pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
	createRGBCloud(cloud_t, target_distances, cloud_rgb);
	//createRGBCloud(cloud_q, self_distances, cloud_rgb);
	centerCloud<pcl::PointXYZRGB>(cloud_rgb);

	//enterViewerLoop(cloud_rgb, normals_t, min_point, .01);
	enterViewerLoop(cloud_rgb, normals_t, target_graph, .05);
	//enterViewerLoop(cloud_rgb, normals_q, query_graph, .05);
	
}

