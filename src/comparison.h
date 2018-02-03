// feature eastimation
#include <pcl/features/fpfh.h>
#include <pcl/features/from_meshes.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/point_types.h>
struct CloudWithNormals {
	pcl::PointCloud<pcl::PointXYZ> cloudPositions;
	pcl::PointCloud<pcl::Normal> cloudNormals;
	template <typename CloudPositions, typename CloudNormals>
	CloudWithNormals(CloudPositions p, CloudNormals n)
	    : cloudPositions(p), cloudNormals(n) {}
};
pcl::PointCloud<pcl::PFHSignature125> computeFeatures_PFH(
    const CloudWithNormals input, float searchRadius) {
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125>
	    pfh;
	pfh.setInputCloud(input.cloudPositions.makeShared());
	pfh.setInputNormals(input.cloudNormals.makeShared());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	pfh.setSearchMethod(tree);
	pfh.setKSearch(100);
	pcl::PointCloud<pcl::PFHSignature125> outputFeatures;
	pfh.compute(outputFeatures);
	return outputFeatures;
}

pcl::PointCloud<pcl::FPFHSignature33> computeFeatures_FPFH(
    const CloudWithNormals input, float searchRadius) {
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
	    fpfh;
	fpfh.setInputCloud(input.cloudPositions.makeShared());
	fpfh.setInputNormals(input.cloudNormals.makeShared());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	fpfh.setSearchMethod(tree);
	fpfh.setKSearch(100);
	pcl::PointCloud<pcl::FPFHSignature33> outputFeatures;
	fpfh.compute(outputFeatures);
	return outputFeatures;
}

float l2FeatureDistance(pcl::PFHSignature125 first,
			pcl::PFHSignature125 second) {
	float distance = 0.0;
	int featureSize = 125;

	for (int i = 0; i < featureSize; ++i)
		distance += pow(first.histogram[i] - second.histogram[i], 2);

	return sqrt(distance);
}

float l2FeatureDistance(pcl::FPFHSignature33 first,
			pcl::FPFHSignature33 second) {
	float distance = 0.0;
	int featureSize = 33;

	for (int i = 0; i < featureSize; ++i)
		distance += pow(first.histogram[i] - second.histogram[i], 2);

	return sqrt(distance);
}

template <typename FeatureType>
std::vector<float> selfFeatureDistance(
    const pcl::PointCloud<FeatureType> &features, int targetIndex) {
	std::vector<float> distances;
	const FeatureType targetFeatureValue = features.points[targetIndex];
	for (int i = 0; i < features.points.size(); ++i) {
		float dist =
		    l2FeatureDistance(targetFeatureValue, features.points[i]);
		distances.push_back(dist);
	}
	return distances;
}


template <typename FeatureType>
std::vector<float> computeFeatureDistancesFromTargetModel(
    pcl::PointCloud<FeatureType> targetFeatures, FeatureType queryFeature
    ) {
std::vector<float> outputDistances;
	for (int i = 0; i < targetFeatures.points.size(); ++i) {
		float dist =
		    l2FeatureDistance(queryFeature, targetFeatures.points[i]);
		outputDistances.push_back(dist);
	}
	return outputDistances;
}