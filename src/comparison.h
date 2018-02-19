#ifndef COMPARISON_HPP
#define COMPARISON_HPP

#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/common/centroid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot_omp.h>
#include <pcl/point_types.h>

#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/impl/search.hpp>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif  // PCL_NO_PRECOMPILE
#include "helpers.hpp"

template <typename F>
float l2FeatureDistance(F first, F second) {
	float dist = 0.0;
	for (int i = 0; i < first.descriptorSize(); ++i)
		dist += pow(first.histogram[i] - second.histogram[i], 2);
	return sqrt(dist);
}

float l2FeatureDistance(pcl::SHOT352 first, pcl::SHOT352 second) {
	float dist = 0.0;
	for (int i = 0; i < first.descriptorSize(); ++i)
		dist += pow(first.descriptor[i] - second.descriptor[i], 2);
	return sqrt(dist);
}

pcl::PointCloud<pcl::PFHSignature125> computeFeaturesPFH(const Mesh input,
							 float searchRadius) {
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125>
	    pfh;
	pfh.setInputCloud(input.positions.makeShared());
	pfh.setInputNormals(input.normals.makeShared());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	pfh.setSearchMethod(tree);
	// pfh.setRadiusSearch(searchRadius);
	pfh.setKSearch(searchRadius);
	pcl::PointCloud<pcl::PFHSignature125> outputFeatures;
	pfh.compute(outputFeatures);
	return outputFeatures;
}

pcl::PointCloud<pcl::PFHSignature125> computePointFeaturesPFH(
    const Mesh input, int idx, float searchRadius) {
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125>
	    pfh;
	pfh.setInputCloud(input.positions.makeShared());
	pfh.setInputNormals(input.normals.makeShared());
	boost::shared_ptr<std::vector<int>> indicesptr(
	    new std::vector<int>{idx});
	pfh.setIndices(indicesptr);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	pfh.setSearchMethod(tree);
	pfh.setRadiusSearch(searchRadius);
	// pfh.setKSearch(searchRadius);
	pcl::PointCloud<pcl::PFHSignature125> outputFeatures;
	pfh.compute(outputFeatures);
	std::cout << "PFH size : " << outputFeatures.points.size() << "\n";
	return outputFeatures;
}

pcl::PointCloud<pcl::FPFHSignature33> computeFeaturesFPFH(const Mesh input,
							  float searchRadius) {
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
	    fpfh;
	fpfh.setInputCloud(input.positions.makeShared());
	fpfh.setInputNormals(input.normals.makeShared());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	fpfh.setSearchMethod(tree);
	fpfh.setKSearch(600);
	pcl::PointCloud<pcl::FPFHSignature33> outputFeatures;
	fpfh.compute(outputFeatures);
	return outputFeatures;
}

pcl::PointCloud<pcl::FPFHSignature33> computePointFeaturesFPFH(const Mesh input,
							       int idx,
							       float radius) {
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
	    fpfh;
	pcl::PointCloud<pcl::FPFHSignature33> fpfhs;

	boost::shared_ptr<std::vector<int>> indicesptr(
	    new std::vector<int>{idx});

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());

	fpfh.setInputCloud(input.positions.makeShared());
	fpfh.setInputNormals(input.normals.makeShared());
	fpfh.setIndices(indicesptr);
	fpfh.setSearchMethod(tree);
	fpfh.setRadiusSearch(radius);
	// fpfh.setKSearch(searchRadius);
	fpfh.compute(fpfhs);

	return fpfhs;
}

pcl::PointCloud<pcl::SHOT352> computeFeaturesSHOT352(const Mesh input,
						     float searchRadius) {
	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	pcl::PointCloud<pcl::SHOT352> outputFeatures;

	shot.setInputCloud(input.positions.makeShared());
	shot.setInputNormals(input.normals.makeShared());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	shot.setSearchMethod(tree);
	shot.setRadiusSearch(searchRadius);
	shot.compute(outputFeatures);
	return outputFeatures;
}

template <typename F>
std::vector<float> selfFeatureDistance(const pcl::PointCloud<F> &features,
				       int idx) {
	std::vector<float> dists;
	F feature_t = features.points[idx];
	for (int i = 0; i < features.points.size(); ++i) {
		float dist = l2FeatureDistance(feature_t, features.points[i]);
		dists.push_back(dist);
	}
	return dists;
}

template <typename F>
std::vector<float> computeFeatureDistancesFromTargetModel(
    pcl::PointCloud<F> features_t, F feature_q) {
	std::vector<float> dists;
	for (int i = 0; i < features_t.points.size(); ++i) {
		float dist = l2FeatureDistance(feature_q, features_t.points[i]);
		dists.push_back(dist);
	}
	return dists;
}

std::vector<float> histDiff(float *first, float *second, int length) {
	std::vector<float> diffs(length);
	for (int i = 0; i < length; ++i) {
		float diff = std::fabs(first[i] - second[i]);
		diffs[i] = diff;
	}
	return diffs;
}

bool checkAngle(pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c,
		float threshold_angle = 90) {
	Eigen::Vector3d b_to_a(a.x - b.x, a.y - b.y, a.z - b.z);
	Eigen::Vector3d b_to_c(c.x - b.x, c.y - b.y, c.z - b.z);

	if (threshold_angle > computeAngle(b_to_a, b_to_c)) return false;

	return true;
}

std::vector<int> findIndices(pcl::PointCloud<pcl::PointXYZ> &cloud,
			     float inner_radius, float outer_radius,
			     std::vector<int> nodes_idx,
			     std::vector<float> &distances) {
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::PointXYZ query_p = cloud.points[nodes_idx.back()];

	kdtree.setInputCloud(cloud.makeShared());

	std::vector<int> points_to_return;
	std::vector<int> points_idx;
	std::vector<float> points_dists;

	int num_of_points = kdtree.radiusSearch(query_p, outer_radius,
						points_idx, points_dists);

	// std::cout << "Found " << num_of_points << " points!\n";

	for (size_t i = 0; i < points_idx.size(); ++i) {
		if (points_dists[i] < pow(inner_radius, 2)) continue;

		if (distances[points_idx[i]] == 255) continue;

		// std::cout << "Nodes idx " << nodes_idx.size() << "!\n";
		if (nodes_idx.size() > 1) {
			// std::cout << "Points to check " <<
			// nodes_idx[nodes_idx.size() - 1] << " " <<
			// nodes_idx[nodes_idx.size()] << "\n";
			if (std::find(nodes_idx.begin(), nodes_idx.end(),
				      points_idx[i]) != nodes_idx.end())
				continue;
			auto a = cloud.points[nodes_idx[nodes_idx.size() - 2]];
			auto b = cloud.points[nodes_idx[nodes_idx.size() - 1]];
			auto c = cloud.points[points_idx[i]];

			// std::cout << "Checing angle!\n";
			if (!checkAngle(a, b, c, 135)) continue;
		}

		points_to_return.push_back(points_idx[i]);
	}

	// std::cout << "After erasing left " << points_to_return.size() << "
	// points!\n";

	return points_to_return;
}

std::vector<pcl::SHOT352> matchIndicesFeatures(
    std::vector<int> idxs, pcl::PointCloud<pcl::SHOT352> features) {
	std::vector<pcl::SHOT352> out_features;
	for (int i = 0; i < idxs.size(); ++i) {
		out_features.push_back(features[idxs[i]]);
	}
	return out_features;
}

std::vector<pcl::PointXYZ> matchIndicesPositions(
    const std::vector<int> &indices,
    const pcl::PointCloud<pcl::PointXYZ> &point_cloud) {
	std::vector<pcl::PointXYZ> output(indices.size());

	for (int index = 0; index < indices.size(); index++) {
		pcl::PointXYZ p = point_cloud.points[indices[index]];
		output[index] = p;
	}
	return output;
}

std::vector<pcl::Normal> matchIndicesNormals(
    const std::vector<int> &indices,
    const pcl::PointCloud<pcl::Normal> &point_cloud) {
	std::vector<pcl::Normal> output(indices.size());

	for (int index = 0; index < indices.size(); index++) {
		pcl::Normal p = point_cloud.points[indices[index]];
		output[index] = p;
	}
	return output;
}

int findClosestFeature(std::vector<pcl::SHOT352> &features,
		       pcl::SHOT352 query_f) {
	int out_index;
	float min_f_dist = 100000;

	for (int i = 0; i < features.size(); i++) {
		auto feature = features[i];
		float f_dist = l2FeatureDistance(feature, query_f);

		if (f_dist < min_f_dist) {
			min_f_dist = f_dist;
			out_index = i;
		}
	}
	return out_index;
}

void thresholdVector(std::vector<float> &v, float w) {
	auto result = minmax_element(v.begin(), v.end());
	float mean =
	    w * (v[result.first - v.begin()] + v[result.second - v.begin()]) /
	    2.0;

	for (int i = 0; i < v.size(); ++i) {
		if (v[i] < mean)
			v[i] = 0;
		else
			v[i] = 255;
	}
}

pcl::SHOT352 computeMeanFeature(pcl::PointCloud<pcl::SHOT352> features,
				std::vector<int> idxs) {
	pcl::SHOT352 mean;
	for (int j = 0; j < 352; ++j) {
		mean.descriptor[j] = 0;
	}
	for (int i = 0; i < idxs.size(); ++i) {
		for (int j = 0; j < 352; ++j) {
			mean.descriptor[j] +=
			    features.points[idxs[i]].descriptor[j];
		}
	}
	for (int j = 0; j < 352; ++j) {
		mean.descriptor[j] /= idxs.size();
	}
	return mean;
}

std::vector<int> createGraph(pcl::PointCloud<pcl::PointXYZ> cloud,
			     pcl::PointCloud<pcl::SHOT352> features,
			     float inner_radius, float outer_radius,
			     std::vector<float> distances, int idx) {
	std::vector<int> nodes_idx{idx};
	std::vector<int> indices_vec;
	std::vector<int> checking_idxs;

	// for (int i = 0; i < 5; ++i) {
	while (1) {
		indices_vec = findIndices(cloud, inner_radius, outer_radius,
					  nodes_idx, distances);
		// std::cout << "Found " << indices_vec.size()  << " in rep " <<
		// i << std::endl;
		if (indices_vec.size() == 0) break;

		auto features_vec = matchIndicesFeatures(indices_vec, features);
		int closestFeature_idx = findClosestFeature(
		    features_vec, features[nodes_idx.back()]);
		nodes_idx.push_back(indices_vec[closestFeature_idx]);
		checking_idxs.insert(checking_idxs.end(), indices_vec.begin(),
				     indices_vec.end());
	}

	//	for (auto i : nodes_idx)
	//		std::cout << "Index of closest feature: " << i  <<
	// std::endl;

	return nodes_idx;
}

bool checkPoint(pcl::PointCloud<pcl::PointXYZ> cloud, int p,
		std::vector<int> graph) {
	int sum = 0;

	for (int i = 0; i < graph.size(); ++i) {
		float dist =
		    pow(cloud.points[p].x - cloud.points[graph[i]].x, 2) +
		    pow(cloud.points[p].y - cloud.points[graph[i]].y, 2) +
		    pow(cloud.points[p].z - cloud.points[graph[i]].z, 2);

		if (dist < pow(0.5, 2)) return false;
	}

	return true;
}

int findPointsWithMinDist(std::vector<std::tuple<int, float>> distances,
			  std::vector<int> graph,
			  pcl::PointCloud<pcl::PointXYZ> cloud) {
	for (int i = 0; i < distances.size(); ++i) {
		if (checkPoint(cloud, std::get<0>(distances[i]), graph))
			return std::get<0>(distances[i]);
	}
	return -1;
}

std::vector<float> computeGraphHist(std::vector<pcl::PointXYZ> positions,
				    std::vector<pcl::Normal> normals,
				    int histogram_size) {
	std::vector<int> vote_hist(histogram_size,
				   0);  // holds the votes for each angle
	float binWidth = 360 / histogram_size;
	for (int i = 1; i < positions.size() - 1; i++) {
		// std::cout << "\n";
		// std::cout << "Node index:" << i << std::endl;
		pcl::PointXYZ a = positions[i - 1];
		pcl::PointXYZ b = positions[i];
		pcl::PointXYZ c = positions[i + 1];
		// std::cout << "a" << a << std::endl;
		// std::cout << "b" << b << std::endl;
		// std::cout << "c" << c << std::endl;
		Eigen::Vector3d a_to_b(b.x - a.x, b.y - a.y, b.z - a.z);
		Eigen::Vector3d b_to_c(c.x - b.x, c.y - b.y, c.z - b.z);
		Eigen::Vector3d n(normals[i].normal_x, normals[i].normal_y,
				  normals[i].normal_z);
		float angle = computeSignedAngle(a_to_b, b_to_c, n);
		// std::cout << "angle:" << angle << std::endl;
		int bin_index = angle / binWidth;
		// std::cout << "bin index:" << bin_index << std::endl;
		vote_hist[bin_index]++;
	}

	std::vector<float> normalized_hist(histogram_size);
	std::transform(vote_hist.begin(), vote_hist.end(),
		       normalized_hist.begin(), [=](int bin_votes) {
			       return bin_votes / float(positions.size() - 2);
		       });

	// std::cout << sum << std::endl;
	return normalized_hist;
}

float computeHistDiff(std::vector<float> h1, std::vector<float> h2) {
	assert(h1.size() == h2.size());

	float output = 0;
	for (int i = 0; i < h1.size(); i++) {
		output += std::pow(h1[i] - h2[i], 2);
	}
	return output;
}

std::vector<int> computeQueryGraph(
    const Mesh &M, const pcl::PointCloud<pcl::SHOT352> &features, int idx,
    float inner_radius, float outer_radius,
    float query_selfDistance_threshold) {
	std::cout << "Computing self distances for query\n";
	std::vector<float> self_distances = selfFeatureDistance(features, idx);
	thresholdVector(self_distances, query_selfDistance_threshold);

	return createGraph(M.positions, features, inner_radius, outer_radius,
			   self_distances, idx);
}

std::vector<std::vector<int>> extractTargetGraphs(
    const Mesh &target, std::vector<float> distances_from_query,
    const pcl::PointCloud<pcl::SHOT352> &features_t, float inner_radius,
    float outer_radius, int number_of_graphs, int min_nodes = 15) {
	std::vector<std::vector<int>> target_graphs;  // output

	std::vector<std::tuple<int, float>> dist_idx;
	for (int i = 0; i < distances_from_query.size(); ++i) {
		dist_idx.push_back(std::make_tuple(i, distances_from_query[i]));
	}
	std::sort(
	    std::begin(dist_idx), std::end(dist_idx),
	    [](std::tuple<int, float> const &t1,
	       std::tuple<int, float> const &t2) {
		    return std::get<1>(t1) <
			   std::get<1>(t2);  // or use a custom compare function

	    });

	std::vector<int> all_nodes_t;
	for (int i = 0; i < number_of_graphs; ++i) {
		int min_point = findPointsWithMinDist(dist_idx, all_nodes_t,
						      target.positions);

		std::cout << "Creating graph " << i << std::endl;
		std::vector<int> temp_nodes =
		    createGraph(target.positions, features_t, inner_radius,
				outer_radius, distances_from_query, min_point);

		std::cout << "Graph found with " << temp_nodes.size() << "\n";
		all_nodes_t.insert(all_nodes_t.end(), temp_nodes.begin(),
				   temp_nodes.end());
		if (temp_nodes.size() < min_nodes) continue;
		target_graphs.push_back(temp_nodes);
	}
	return target_graphs;
}

std::vector<std::pair<int, float>> computeSortedTargetGraphs(
    const Mesh &query, const std::vector<int> &query_graph, const Mesh &target,
    const std::vector<std::vector<int>> &target_graphs) {
	int hist_size = 50;
	std::vector<pcl::PointXYZ> node_positions =
	    matchIndicesPositions(query_graph, target.positions);
	std::vector<pcl::Normal> node_normals =
	    matchIndicesNormals(query_graph, target.normals);
	std::vector<float> angle_hist_q =
	    computeGraphHist(node_positions, node_normals, hist_size);

	std::vector<std::pair<int, float>> hist_differences(
	    target_graphs.size());
	for (int i = 0; i < target_graphs.size(); i++) {
		const std::vector<int> &graph_t = target_graphs[i];
		node_positions =
		    matchIndicesPositions(graph_t, target.positions);
		node_normals = matchIndicesNormals(graph_t, target.normals);
		std::vector<float> angle_hist_t =
		    computeGraphHist(node_positions, node_normals, hist_size);
		float hist_diff = computeHistDiff(angle_hist_q, angle_hist_t);
		std::cout << "Graph with index " << i << " has diff "
			  << hist_diff << std::endl;
		hist_differences[i] = std::make_pair(i, hist_diff);
	}
	std::sort(std::begin(hist_differences), std::end(hist_differences),
		  [](std::pair<int, float> const &t1,
		     std::pair<int, float> const &t2) {
			  return t1.second < t2.second;

		  });
	return hist_differences;
}

#endif  // COMPARISON_HPP
