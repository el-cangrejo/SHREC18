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
	auto features_q =
	    computeFeaturesSHOT352(cloud_qWithNormals, inner_radius);

	pcl::fromPCLPointCloud2(mesh_t.cloud, cloud_t);
	std::cout << "Target model : " << cloud_t.points.size() << "\n";

	computeNormals(mesh_t, cloud_t, normals_t);
	CloudWithNormals cloud_tWithNormals(cloud_t, normals_t);
	std::cout << "Computing target features\n";
	auto features_t =
	    computeFeaturesSHOT352(cloud_tWithNormals, inner_radius);

	std::cout << "Computing self distances for query\n";
	std::vector<float> self_distances =
	    selfFeatureDistance(features_q, idx);
	thresholdVector(self_distances, atof(argv[6]));

	int N = atoi(argv[5]);
	std::vector<int> query_graph =
	    createGraph(cloud_q, features_q, 0.06, 0.12, self_distances, idx);

	pcl::SHOT352 mean_feature = computeMeanFeature(features_q, query_graph);

	std::cout << "Computing distances to target\n";
	std::vector<float> target_distances =
	    computeFeatureDistancesFromTargetModel<pcl::SHOT352>(
		features_t, features_q.points[idx]);
	// computeFeatureDistancesFromTargetModel<pcl::SHOT352>(features_t,
	// mean_feature);
	thresholdVector(target_distances, atof(argv[6]));

	std::vector<int> target_graph;
	std::vector<std::vector<int>> target_graph_vis;

	std::vector<std::tuple<int, float>> dist_idx;
	for (int i = 0; i < target_distances.size(); ++i) {
		dist_idx.push_back(std::make_tuple(i, target_distances[i]));
	}

	std::sort(
	    std::begin(dist_idx), std::end(dist_idx),
	    [](std::tuple<int, float> const &t1,
	       std::tuple<int, float> const &t2) {
		    return std::get<1>(t1) <
			   std::get<1>(t2);  // or use a custom compare function
	    });

	for (int i = 0; i < atoi(argv[8]); ++i) {
		int min_point =
		    findPointsWithMinDist(dist_idx, target_graph, cloud_t);

		std::cout << "Creating graph " << i << std::endl;
		std::vector<int> temp_nodes =
		    createGraph(cloud_t, features_t, inner_radius, outer_radius,
				target_distances, min_point);

		std::cout << "Graph found with " << temp_nodes.size() << "\n";
		target_graph.insert(target_graph.end(), temp_nodes.begin(),
				    temp_nodes.end());
		if (temp_nodes.size() < 15) continue;
		target_graph_vis.push_back(temp_nodes);
	}

	centerCloud<pcl::PointXYZ>(cloud_t);  // rgb cloud will also be centered

	// compute querry histogram
	int hist_size = 10;
	std::vector<pcl::PointXYZ> node_positions =
	    matchIndicesPositions(query_graph, cloud_t);
	std::vector<pcl::Normal> node_normals =
	    matchIndicesNormals(query_graph, normals_t);
	std::vector<float> angle_hist_q =
	    computeGraphHist(node_positions, node_normals, hist_size);

	std::vector<std::tuple<int, float>> hist_differences(
	    target_graph_vis.size());

	// populate hist_differences
	for (int i = 0; i < target_graph_vis.size(); i++) {
		const std::vector<int> &graph_t = target_graph_vis[i];
		node_positions = matchIndicesPositions(graph_t, cloud_t);
		node_normals = matchIndicesNormals(graph_t, normals_t);
		// using Point = pcl::PointXYZ;
		// using Normal = pcl::Normal;
		// std::vector<Normal> n{Normal(0, 0, 1), Normal(0, 0, 1),
		// Normal(0, 0,
		// 1),
		//		      Normal(0, 0, 1), Normal(0, 0, 1)};
		// std::vector<Point> v{Point(0, 1, 0), Point(1, 1, 0), Point(2,
		// 2, 0),
		//		     Point(3, 2, 0), Point(3, 3, 0)};
		std::vector<float> angle_hist_t =
		    // computeGraphHist(node_positions, hist_size);
		    computeGraphHist(node_positions, node_normals, hist_size);

		float hist_diff = computeHistDiff(angle_hist_q, angle_hist_t);
		std::cout << "Graph with index " << i << " has diff "
			  << hist_diff << std::endl;
		hist_differences[i] = std::make_tuple(i, hist_diff);
	}

	std::sort(
	    std::begin(hist_differences), std::end(hist_differences),
	    [](std::tuple<int, float> const &t1,
	       std::tuple<int, float> const &t2) {
		    return std::get<1>(t1) <
			   std::get<1>(t2);  // or use a custom compare function
	    });
	std::vector<std::vector<int>> closest_graphs_vis;
	assert(hist_differences.size() >= 3);
	for (int i = 0; i < 3; i++) {
		closest_graphs_vis.push_back(
		    target_graph_vis[std::get<0>(hist_differences[i])]);
	}
	std::cout << "closest_graphs_vis.size()=" << closest_graphs_vis.size()
		  << std::endl;
	// std::vector<std::vector<int>> closest_graphs_vis(
	//    {target_graph_vis[std::get<0>(hist_differences[0])],
	//     target_graph_vis[std::get<0>(hist_differences[1])],
	//     target_graph_vis[std::get<0>(hist_differences[2])]});

	pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
	// centerCloud<pcl::PointXYZRGB>(cloud_rgb);
	createRGBCloud(cloud_t, target_distances, cloud_rgb);
	// createRGBCloud(cloud_q, self_distances, cloud_rgb);
	centerCloud<pcl::PointXYZRGB>(cloud_rgb);
	centerCloud<pcl::PointXYZ>(cloud_t);

	// enterViewerLoop(cloud_rgb, normals_t, min_point, .01);
	enterViewerLoop(mesh_t, cloud_t, normals_t, closest_graphs_vis, .05);
	// enterViewerLoop(cloud_rgb, normals_q, query_graph, .05);
}
