#include <pcl/features/boundary.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <iostream>
#include <string>
#include <tuple>
#include "comparison.h"
#include "helpers.hpp"
#include "viewer.hpp"

int main(int argc, char **argv) {
	std::cout << "Hello SHREC2018!\n";

	std::string query_filename = "shrec18_recognition/Queries/" +
				     std::to_string(atoi(argv[1])) + ".ply";
	std::string target_filename = "shrec18_recognition/Dataset/" +
				      std::to_string(atoi(argv[7])) + ".ply";

	Mesh query = load(query_filename);

	Mesh target = load(target_filename);

	float inner_radius = atof(argv[3]);
	std::cout << "Computing target features\n";
	auto features_t = computeFeaturesSHOT352(target, inner_radius);

	std::cout << "Computing query features\n";
	auto features_q = computeFeaturesSHOT352(query, inner_radius);

	int idx = atoi(argv[2]);

	std::vector<int> query_graph = computeQueryGraph(
	    query, features_q, idx, 0.06, 0.12, atof(argv[6]));

	pcl::SHOT352 mean_feature = computeMeanFeature(features_q, query_graph);

	std::cout << "Computing distances to target\n";
	std::vector<float> target_distances =
	    computeFeatureDistancesFromTargetModel<pcl::SHOT352>(
		features_t, features_q.points[idx]);
	// computeFeatureDistancesFromTargetModel<pcl::SHOT352>(features_t,
	// mean_feature);
	thresholdVector(target_distances, atof(argv[6]));

	float outer_radius = atof(argv[4]);
	int number_of_graphs = atoi(argv[8]);
	std::vector<std::vector<int>> target_graphs =
	    extractTargetGraphs(target, target_distances, features_t,
				inner_radius, outer_radius, number_of_graphs);

	centerCloud<pcl::PointXYZ>(
	    target.positions);  // rgb cloud will also be centered

	std::vector<std::pair<int, float>> sorted_graphs_t =
	    computeSortedTargetGraphs(query, query_graph, target,
				      target_graphs);

	std::vector<std::vector<int>> closest_graphs_vis;
	int vis_graph_number = 10;
	assert(sorted_graphs_t.size() >= vis_graph_number);
	for (int i = 0; i < vis_graph_number; i++) {
		closest_graphs_vis.push_back(
		    target_graphs[std::get<0>(sorted_graphs_t[i])]);
	}
	std::cout << "closest_graphs_vis.size()=" << closest_graphs_vis.size()
		  << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
	createRGBCloud(target.positions, target_distances, cloud_rgb);
	// createRGBCloud(cloud_q, self_distances, cloud_rgb);
	centerCloud<pcl::PointXYZRGB>(cloud_rgb);
	centerCloud<pcl::PointXYZ>(target.positions);

	// enterViewerLoop(cloud_rgb, normals_t, min_point, .01);
	enterViewerLoop(target.mesh, target.positions, target.normals,
			closest_graphs_vis, .05);
	// enterViewerLoop(cloud_rgb, normals_q, query_graph, .05);
}
