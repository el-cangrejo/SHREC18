#ifndef HELPERS_HPP
#define HELPERS_HPP 

#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/conversions.h>
#include <pcl/features/shot.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/features/shot.h>


void computeNormals(const pcl::PolygonMesh &mesh,
		    pcl::PointCloud<pcl::PointXYZ> &cloud,
		    pcl::PointCloud<pcl::Normal> &normals)
{
  auto polygons = mesh.polygons;
  int nr_points = static_cast<int>(cloud.points.size());
  int nr_polygons = static_cast<int>(polygons.size());

  normals.header = cloud.header;
  normals.width = cloud.width;
  normals.height = cloud.height;
  normals.points.resize(nr_points);

  for (int i = 0; i < nr_points; ++i)
    normals.points[i].getNormalVector3fMap() = Eigen::Vector3f::Zero();

  // NOTE: for efficiency the weight is computed implicitly by using the
  // cross product, this causes inaccurate normals for meshes containing
  // non-triangle polygons (quads or other types)
  for (int i = 0; i < nr_polygons; ++i) {
    const int nr_points_polygon = (int)polygons[i].vertices.size();
    if (nr_points_polygon < 3) continue;

    // compute normal for triangle
    Eigen::Vector3f vec_a_b =
	cloud.points[polygons[i].vertices[0]].getVector3fMap() -
	cloud.points[polygons[i].vertices[1]].getVector3fMap();
    Eigen::Vector3f vec_a_c =
	cloud.points[polygons[i].vertices[0]].getVector3fMap() -
	cloud.points[polygons[i].vertices[2]].getVector3fMap();
    Eigen::Vector3f normal = vec_a_b.cross(vec_a_c);
    //    pcl::flipNormalTowardsViewpoint(cloud.points[polygons[i].vertices[0]],
    //    0.0f, 0.0f, 0.0f, normal(0), normal(1), normal(2));

    // add normal to all points in polygon
    for (int j = 0; j < nr_points_polygon; ++j)
      normals.points[polygons[i].vertices[j]].getNormalVector3fMap() += normal;
  }

  for (int i = 0; i < nr_points; ++i) {
    normals.points[i].getNormalVector3fMap().normalize();
    //    pcl::flipNormalTowardsViewpoint(cloud.points[i], 0.0f,
    //    0.0f, 0.0f, normals.points[i].normal_x,
    //    normals.points[i].normal_y, normals.points[i].normal_z);
  }
}

template <typename T>
void centerCloud(pcl::PointCloud<T> &cloud)
{
  pcl::CentroidPoint<T> centroid;
  for (int i = 0; i < cloud.points.size(); ++i)
    centroid.add(cloud.points[i]);
  
  T c;
  centroid.get(c);

  std::cout << "Centroid: " << c << "\n";
  for (int i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = cloud.points[i].x - c.x;
    cloud.points[i].y = cloud.points[i].y - c.y;
    cloud.points[i].z = cloud.points[i].z - c.z;
  }
}

void fitToUnitCloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  float max_dist = 0.0;

  for (int i = 0; i < cloud.points.size(); ++i) {
    float dist = sqrt(pow(cloud.points[i].x, 2) + pow(cloud.points[i].y, 2) +
		      pow(cloud.points[i].z, 2));

    if (dist > max_dist) max_dist = dist;
  }

  for (int i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x /= max_dist;
    cloud.points[i].y /= max_dist;
    cloud.points[i].z /= max_dist;
  }
}

void createRGBCloud(const pcl::PointCloud<pcl::PointXYZ> &input,
		    std::vector<float> dists,
		    pcl::PointCloud<pcl::PointXYZRGB> &output)
{

  auto min_max = std::minmax_element(dists.begin(), dists.end());

  float min_dist = dists[min_max.first - dists.begin()];
  float max_dist = dists[min_max.second - dists.begin()];
  float range = max_dist - min_dist;

  pcl::copyPointCloud(input, output);

  for (int i = 0; i < dists.size(); i++)
    output[i].r = 255 * dists[i] / range;
}

#endif // HELPERS_HPP
