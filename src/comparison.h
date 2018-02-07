#ifndef COMPARISON_HPP
#define COMPARISON_HPP

#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot_omp.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/impl/search.hpp>

#ifndef PCL_NO_PRECOMPILE
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
	//pfh.setRadiusSearch(searchRadius);
	pfh.setKSearch(searchRadius);
	pcl::PointCloud<pcl::PFHSignature125> outputFeatures;
	pfh.compute(outputFeatures);
	return outputFeatures;
}

pcl::PointCloud<pcl::PFHSignature125> computePointFeaturesPFH(
    const CloudWithNormals input, int idx, float searchRadius) {
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(input.cloudPositions.makeShared());
	pfh.setInputNormals(input.cloudNormals.makeShared());
	boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> {idx});
	pfh.setIndices(indicesptr);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pfh.setSearchMethod(tree);
	pfh.setRadiusSearch(searchRadius);
	//pfh.setKSearch(searchRadius);
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

pcl::PointCloud<pcl::FPFHSignature33> computePointFeaturesFPFH(
    const CloudWithNormals input, int idx, float radius) {

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	pcl::PointCloud<pcl::FPFHSignature33> fpfhs;

	boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> {idx});

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	fpfh.setInputCloud(input.cloudPositions.makeShared());
	fpfh.setInputNormals(input.cloudNormals.makeShared());
	fpfh.setIndices(indicesptr);
	fpfh.setSearchMethod(tree);
	fpfh.setRadiusSearch(radius);
	//fpfh.setKSearch(searchRadius);
	fpfh.compute(fpfhs);

	return fpfhs;
}

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
}

template <typename F>
std::vector<float> selfFeatureDistance(const pcl::PointCloud<F> &features, int idx) {
	std::vector<float> dists;
	F feature_t = features.points[idx];
	for (int i = 0; i < features.points.size(); ++i) {
		float dist = l2FeatureDistance(feature_t, features.points[i]);
		dists.push_back(dist);
	}
	return dists;
}

template <typename F>
std::vector<float> computeFeatureDistancesFromTargetModel(pcl::PointCloud<F> features_t, F feature_q) {
	std::vector<float> dists;
	for (int i = 0; i < features_t.points.size(); ++i) {
		float dist = l2FeatureDistance(feature_q, features_t.points[i]);
		dists.push_back(dist);
	}
	return dists;
}

std::vector<float> histDiff(float *first, float *second, int length) {
	std::vector<float> diffs(length);
	for(int i = 0; i < length; ++i) {
		float diff = std::fabs(first[i] - second[i]);
		diffs[i] = diff;
	}
	return diffs;
}

void minDistPoint(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::SHOT352> &features, float inner_radius, float outer_radius, int idx_q, int idx_t) {
}
#endif // COMPARISON_HPP
