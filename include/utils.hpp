#ifndef UTILS_HPP
#define UTILS_HPP

#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/from_meshes.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/impl/search.hpp>
//#include <pcl/features.h>

// feature eastimation
#include <pcl/features/pfh.h>
#include <pcl/point_types.h>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif  // PCL_NO_PRECOMPILE

void computeNormals(const pcl::PolygonMesh &mesh,
		    pcl::PointCloud<pcl::PointXYZ> &cloud,
		    pcl::PointCloud<pcl::Normal> &normals);

void centerCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

void normalizeCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

void enterViewerLoop(pcl::PointCloud<pcl::PointXYZ> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals);

void enterViewerLoop(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals,
		     pcl::PointXYZ sphereCenter, float sphereRadius);

void enterViewerLoopMesh(pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			 pcl::PointCloud<pcl::Normal> &normals);

void computeApproximateNormals_(const pcl::PointCloud<pcl::PointXYZ> &cloud,
				const std::vector<pcl::Vertices> &polygons,
				pcl::PointCloud<pcl::Normal> &normals);

// template <typename FeatureType>
void computeFeatures(const pcl::PointCloud<pcl::PointXYZ> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals,
		     pcl::PointCloud<pcl::PFHSignature125> &features,
		     float searchRadius);

void computeFeatures(const pcl::PointCloud<pcl::PointXYZ> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals,
		     pcl::PointCloud<pcl::FPFHSignature33> &features,
		     float searchRadius);

float l2FeatureDistance(pcl::PFHSignature125 first,
			pcl::PFHSignature125 second);

float l2FeatureDistance(pcl::FPFHSignature33 first,
			pcl::FPFHSignature33 second);

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

void createRGBCloud(const pcl::PointCloud<pcl::PointXYZ> &input,
		    std::vector<float> distances,
		    pcl::PointCloud<pcl::PointXYZRGB> &output);
#endif
