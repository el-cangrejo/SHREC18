#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/PolygonMesh.h>
#include <pcl/features/from_meshes.h>
//#include <pcl/features.h>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE


void computeNormals (const pcl::PolygonMesh::Ptr &mesh, pcl::PointCloud<pcl::Normal>::Ptr &normals);

void centerCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

void normalizeCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

void enterViewerLoop(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> normals);
