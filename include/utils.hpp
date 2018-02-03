#ifndef UTILS_HPP
#define UTILS_HPP

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


void createRGBCloud(const pcl::PointCloud<pcl::PointXYZ> &input,
		    std::vector<float> distances,
		    pcl::PointCloud<pcl::PointXYZRGB> &output);
#endif
