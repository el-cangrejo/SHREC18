#ifndef COMPARISON_HPP
#define COMPARISON_HPP

#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/common/centroid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot_omp.h>
<<<<<<< HEAD
=======
#include <pcl/point_types.h>

#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/impl/search.hpp>

#ifndef PCL_NO_PRECOMPILE
>>>>>>> 0d24adae225abf02147dca233fe4ec1d3ca8ff3a
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif  // PCL_NO_PRECOMPILE

struct CloudWithNormals {
	pcl::PointCloud<pcl::PointXYZ> cloudPositions;
	pcl::PointCloud<pcl::Normal> cloudNormals;
	template <typename CloudPositions, typename CloudNormals>
	CloudWithNormals(CloudPositions p, CloudNormals n)
	    : cloudPositions(p), cloudNormals(n) {}
};

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

pcl::PointCloud<pcl::PFHSignature125> computeFeaturesPFH(
    const CloudWithNormals input, float searchRadius) {
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125>
	    pfh;
	pfh.setInputCloud(input.cloudPositions.makeShared());
	pfh.setInputNormals(input.cloudNormals.makeShared());
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
    const CloudWithNormals input, int idx, float searchRadius) {
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125>
	    pfh;
	pfh.setInputCloud(input.cloudPositions.makeShared());
	pfh.setInputNormals(input.cloudNormals.makeShared());
	boost::shared_ptr<std::vector<int> > indicesptr(
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

pcl::PointCloud<pcl::FPFHSignature33> computeFeaturesFPFH(
    const CloudWithNormals input, float searchRadius) {
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
	    fpfh;
	fpfh.setInputCloud(input.cloudPositions.makeShared());
	fpfh.setInputNormals(input.cloudNormals.makeShared());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	fpfh.setSearchMethod(tree);
	fpfh.setKSearch(600);
	pcl::PointCloud<pcl::FPFHSignature33> outputFeatures;
	fpfh.compute(outputFeatures);
	return outputFeatures;
}

<<<<<<< HEAD
pcl::PointCloud<pcl::SHOT352> computeFeatures_SHOT352(
    const CloudWithNormals input, float searchRadius) {
	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> fpfh;
	fpfh.setInputCloud(input.cloudPositions.makeShared());
	fpfh.setInputNormals(input.cloudNormals.makeShared());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	fpfh.setSearchMethod(tree);
	fpfh.setKSearch(0);
	fpfh.setRadiusSearch(searchRadius);
	pcl::PointCloud<pcl::SHOT352> outputFeatures;
	fpfh.compute(outputFeatures);
	return outputFeatures;
}

template <typename FeatureType, typename FeatureSize>
float l2FeatureDistance(FeatureType first, FeatureType second) {
	float distance = 0.0;
	int featureSize = FeatureType::featureSize();
=======
pcl::PointCloud<pcl::FPFHSignature33> computePointFeaturesFPFH(
    const CloudWithNormals input, int idx, float radius) {
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
	    fpfh;
	pcl::PointCloud<pcl::FPFHSignature33> fpfhs;

	boost::shared_ptr<std::vector<int> > indicesptr(
	    new std::vector<int>{idx});

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());

	fpfh.setInputCloud(input.cloudPositions.makeShared());
	fpfh.setInputNormals(input.cloudNormals.makeShared());
	fpfh.setIndices(indicesptr);
	fpfh.setSearchMethod(tree);
	fpfh.setRadiusSearch(radius);
	// fpfh.setKSearch(searchRadius);
	fpfh.compute(fpfhs);
>>>>>>> 0d24adae225abf02147dca233fe4ec1d3ca8ff3a

	return fpfhs;
}

<<<<<<< HEAD
float l2FeatureDistance(pcl::SHOT352 first, pcl::SHOT352 second) {
	float distance = 0.0;
	int featureSize = 352;
	for (int i = 0; i < featureSize; ++i)
		distance += pow(first.descriptor[i] - second.descriptor[i], 2);
	return sqrt(distance);
=======
pcl::PointCloud<pcl::SHOT352> computeFeaturesSHOT352(
    const CloudWithNormals input, float searchRadius) {
	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	pcl::PointCloud<pcl::SHOT352> outputFeatures;

	shot.setInputCloud(input.cloudPositions.makeShared());
	shot.setInputNormals(input.cloudNormals.makeShared());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	shot.setSearchMethod(tree);
	shot.setRadiusSearch(searchRadius);
	shot.compute(outputFeatures);
	return outputFeatures;
>>>>>>> 0d24adae225abf02147dca233fe4ec1d3ca8ff3a
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

<<<<<<< HEAD
template <typename FeatureType>
std::vector<float> computeFeatureDistancesFromTargetModel(
    pcl::PointCloud<FeatureType> targetFeatures, FeatureType queryFeature) {
	std::vector<float> outputDistances(targetFeatures.points.size());
	for (int i = 0; i < targetFeatures.points.size(); ++i) {
		float dist =
		    l2FeatureDistance(queryFeature, targetFeatures.points[i]);
		outputDistances.push_back(dist);
=======
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
>>>>>>> 0d24adae225abf02147dca233fe4ec1d3ca8ff3a
	}
	return diffs;
}

<<<<<<< HEAD
std::vector<float> histogramDifference(float *first, float *second,
				       int histogramSize) {
	std::vector<float> output(histogramSize);

	for (int binIndex = 0; binIndex < histogramSize; binIndex++) {
		float binDifference =
		    std::abs(first[binIndex] - second[binIndex]);
		output[binIndex] = binDifference;
=======
void minDistPoint(std::vector<pcl::SHOT352> &features, pcl::SHOT352 query_f,
		  int &idx_t) {
	float min_f_dist = 100000;

	for (int i = 0; i < features.size(); i++) {
		auto feature = features[i];
		float f_dist = l2FeatureDistance(feature, query_f);

		if (f_dist < min_f_dist) {
			min_f_dist = f_dist;
			idx_t = i;
		}
		std::cout << "Min distance point " << idx_t << "\n";
>>>>>>> 0d24adae225abf02147dca233fe4ec1d3ca8ff3a
	}
}
std::vector<int> findIndices(pcl::PointCloud<pcl::PointXYZ> &cloud, int idx,
			     float inner_radius, float outer_radius) {
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::PointXYZ query_p = cloud.points[idx];

<<<<<<< HEAD
pcl::PointCloud<pcl::PFHSignature125> computePointFeatures_PFH(
    const CloudWithNormals input, int idx, float searchRadius) {
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125>
	    pfh;
	pfh.setInputCloud(input.cloudPositions.makeShared());
	pfh.setInputNormals(input.cloudNormals.makeShared());
	boost::shared_ptr<std::vector<int> > indicesptr(
	    new std::vector<int>{idx});
	pfh.setIndices(indicesptr);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	pfh.setSearchMethod(tree);
	// pfh.setRadiusSearch(searchRadius);
	pfh.setKSearch(searchRadius);
	pcl::PointCloud<pcl::PFHSignature125> outputFeatures;
	pfh.compute(outputFeatures);
	return outputFeatures;
}

pcl::PointCloud<pcl::SHOT352> computePointFeatures_SHOT(
    const CloudWithNormals input, int idx, float searchRadius) {
	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> pfh;
	pfh.setInputCloud(input.cloudPositions.makeShared());
	pfh.setInputNormals(input.cloudNormals.makeShared());
	boost::shared_ptr<std::vector<int> > indicesptr(
	    new std::vector<int>{idx});
	pfh.setIndices(indicesptr);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	pfh.setSearchMethod(tree);
	// pfh.setRadiusSearch(searchRadius);
	pfh.setKSearch(0);
	pfh.setRadiusSearch(searchRadius);
	pcl::PointCloud<pcl::SHOT352> outputFeatures;
	pfh.compute(outputFeatures);
	return outputFeatures;
}
=======
	kdtree.setInputCloud(cloud.makeShared());

	std::vector<int> points_idx;
	std::vector<float> points_dists;

	std::cout << "Found "
		  << kdtree.radiusSearch(query_p, outer_radius, points_idx,
					 points_dists)
		  << " points!\n";

	for (size_t i = 0; i < points_idx.size(); ++i) {
		if (points_dists[i] > inner_radius) continue;

		points_idx.erase(points_idx.begin() + i);
		points_dists.erase(points_dists.begin() + i);
	}
	// std::cout << "After erasing left " << points_idx.size() << "
	// points!\n";

	return points_idx;
}

pcl::PointCloud<pcl::SHOT352> matchIndicesFeatures(
    std::vector<int> idxs, pcl::PointCloud<pcl::SHOT352> features) {
	pcl::PointCloud<pcl::SHOT352> out_features;
	for (int i = 0; i < idxs.size(); ++i) {
		out_features.push_back(features[idxs[i]]);
	}
	return out_features;
}
#endif  // COMPARISON_HPP
>>>>>>> 0d24adae225abf02147dca233fe4ec1d3ca8ff3a
