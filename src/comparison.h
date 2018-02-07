// feature eastimation
#include <pcl/features/fpfh.h>
#include <pcl/features/from_meshes.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot_omp.h>
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
	// pfh.setRadiusSearch(searchRadius);
	pfh.setKSearch(searchRadius);
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
	fpfh.setKSearch(600);
	pcl::PointCloud<pcl::FPFHSignature33> outputFeatures;
	fpfh.compute(outputFeatures);
	return outputFeatures;
}

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

	for (int i = 0; i < featureSize; ++i)
		distance += pow(first.histogram[i] - second.histogram[i], 2);
	return sqrt(distance);
}

float l2FeatureDistance(pcl::SHOT352 first, pcl::SHOT352 second) {
	float distance = 0.0;
	int featureSize = 352;
	for (int i = 0; i < featureSize; ++i)
		distance += pow(first.descriptor[i] - second.descriptor[i], 2);
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
    pcl::PointCloud<FeatureType> targetFeatures, FeatureType queryFeature) {
	std::vector<float> outputDistances(targetFeatures.points.size());
	for (int i = 0; i < targetFeatures.points.size(); ++i) {
		float dist =
		    l2FeatureDistance(queryFeature, targetFeatures.points[i]);
		outputDistances.push_back(dist);
	}
	return outputDistances;
}

std::vector<float> histogramDifference(float *first, float *second,
				       int histogramSize) {
	std::vector<float> output(histogramSize);

	for (int binIndex = 0; binIndex < histogramSize; binIndex++) {
		float binDifference =
		    std::abs(first[binIndex] - second[binIndex]);
		output[binIndex] = binDifference;
	}
	return output;
}

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
