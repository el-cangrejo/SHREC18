// feature eastimation
#include <pcl/features/pfh.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/from_meshes.h>
#include <pcl/features/normal_3d.h>
void computeFeatures(const pcl::PointCloud<pcl::PointXYZ> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals,
		     pcl::PointCloud<pcl::PFHSignature125> &features,
		     float searchRadius) {
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125>
	    pfh;
	pfh.setInputCloud(cloud.makeShared());
	pfh.setInputNormals(normals.makeShared());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	pfh.setSearchMethod(tree);
	pfh.setKSearch(100);
	pfh.compute(features);
}

void computeFeatures(const pcl::PointCloud<pcl::PointXYZ> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals,
		     pcl::PointCloud<pcl::FPFHSignature33> &features,
		     float searchRadius) {
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(cloud.makeShared());
	fpfh.setInputNormals(normals.makeShared());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	    new pcl::search::KdTree<pcl::PointXYZ>());
	fpfh.setSearchMethod(tree);
	fpfh.setKSearch(100);
	fpfh.compute(features);
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

template <typename F>
void computeFeatureDistancesFromTarget(
    pcl::PointCloud<F> pfh, int targetIndex,
    std::vector<float> &distances) {
	const F targetFeatureValue = pfh.points[targetIndex];
	for (int i = 0; i < pfh.points.size(); ++i) {
		float dist =
		    l2FeatureDistance(targetFeatureValue, pfh.points[i]);
		distances.push_back(dist);
	}
}
