#ifndef UTILS_HPP
#define UTILS_HPP

#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/from_meshes.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/features.h>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE

void computeNormals(const pcl::PolygonMesh &mesh,
		    pcl::PointCloud<pcl::PointXYZ> &cloud,
		    pcl::PointCloud<pcl::Normal> &normals);

void centerCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

void normalizeCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

void enterViewerLoop(pcl::PointCloud<pcl::PointXYZ> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals);

void enterViewerLoopMesh(pcl::PolygonMesh &mesh,
		     pcl::PointCloud<pcl::Normal> &normals);

void computeApproximateNormals_(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::vector<pcl::Vertices>& polygons, pcl::PointCloud<pcl::Normal>& normals);

float l2FeatureDistance (pcl::PFHSignature125 first, pcl::PFHSignature125 second);

#endif
