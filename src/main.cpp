#include <iostream>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/visualization/cloud_viewer.h>

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

int main(int argc, char **argv)
{
  std::cout << "Hello SHREC2018!\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  std::string queryModel = "shrec18_recognition/Queries/2.ply";
  std::string dataModel = "shrec18_recognition/Dataset/2.ply";
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(queryModel, *cloud) == -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << cloud->points.size() << " data points" << std::endl;

  auto cloud_normals = execute_normalEstimation(cloud);
  std::cout << "Computed " << cloud_normals->points.size() << " normals" << std::endl;

  auto fpfhs = execute_featureEstimation(cloud, cloud_normals);
  std::cout << "Computed " << fpfhs->points.size() << " features" << std::endl;

  enterViewerLoop(cloud, cloud_normals);
}
