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

// pcl::PointCloud<pcl::SHOT352> computeFeatures_SHOT352(
//     const CloudWithNormals input, float searchRadius) {
// 	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> fpfh;
// 	fpfh.setInputCloud(input.cloudPositions.makeShared());
// 	fpfh.setInputNormals(input.cloudNormals.makeShared());
// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
// 	    new pcl::search::KdTree<pcl::PointXYZ>());
// 	fpfh.setSearchMethod(tree);
// 	fpfh.setRadiusSearch(0.3);
// 	pcl::PointCloud<pcl::SHOT352> outputFeatures;
// 	fpfh.compute(outputFeatures);
// 	return outputFeatures;
// }

template <typename FeatureType, typename FeatureSize>
float l2FeatureDistance(FeatureType first, FeatureType second,
			FeatureSize featureSize) {
	float distance = 0.0;

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

template <typename FeatureType, typename FeatureSize>
std::vector<float> computeFeatureDistancesFromTargetModel(
    pcl::PointCloud<FeatureType> targetFeatures, FeatureType queryFeature,
    FeatureSize featureSize) {
	std::vector<float> outputDistances;
	for (int i = 0; i < targetFeatures.points.size(); ++i) {
		float dist = l2FeatureDistance(
		    queryFeature, targetFeatures.points[i], featureSize);
		outputDistances.push_back(dist);
	}
	return outputDistances;
}
