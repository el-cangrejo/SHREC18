#include <iostream>
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

pcl::PointCloud<pcl::Normal>::Ptr
execute_normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.03);
  ne.compute(*cloud_normals);

  return cloud_normals;
}

pcl::PointCloud<pcl::Normal>::Ptr computeNormals (const pcl::PolygonMesh::Ptr &mesh) {
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

	pcl::features::computeApproximateNormals(mesh->cloud, mesh->polygons, normals);
}



template <typename Cloud, typename CloudNormals>
pcl::PointCloud<pcl::FPFHSignature33>::Ptr execute_featureEstimation(Cloud cloud, CloudNormals normals)
{
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(cloud);
  fpfh.setInputNormals(normals);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  fpfh.setSearchMethod(tree);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(
      new pcl::PointCloud<pcl::FPFHSignature33>());

  fpfh.setRadiusSearch(0.05);
  fpfh.compute(*fpfhs);

  return fpfhs;
}

template <typename Cloud, typename CloudNormals>
void enterViewerLoop(Cloud cloud, CloudNormals normals)
{
  pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
  viewer.setBackgroundColor(0, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10,
							  0.05, "normals");
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }
}

void normalizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
	pcl::CentroidPoint<pcl::PointXYZ> centroid;	
	for (int i = 0; i < cloud->points.size(); ++i) {
		centroid.add(cloud->points[i]);
	}
	pcl::PointXYZ c;
	centroid.get(c);

	for (int i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = cloud->points[i].x - c.x;
		cloud->points[i].y = cloud->points[i].y - c.y;
		cloud->points[i].z = cloud->points[i].z - c.z;
	}
}

int main(int argc, char **argv)
{
  std::cout << "Hello SHREC2018!\n";

  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

  std::string queryModel = "shrec18_recognition/Queries/2.ply";
  std::string dataModel = "shrec18_recognition/Dataset/2.ply";
  if (pcl::io::loadPLYFile(queryModel, *mesh) == -1) {
    return -1;
  }

  std::cout << "Loaded " << mesh->cloud.data.size() << " data points" << std::endl;

	//normalizeCloud(cloud);

  //auto cloud_normals = execute_normalEstimation(cloud);
  //std::cout << "Computed " << cloud_normals->points.size() << " normals" << std::endl;

  //auto fpfhs = execute_featureEstimation(cloud, cloud_normals);
  //std::cout << "Computed " << fpfhs->points.size() << " features" << std::endl;

  //enterViewerLoop(cloud, cloud_normals);
}
