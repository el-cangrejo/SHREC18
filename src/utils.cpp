#include "utils.hpp"

void computeNormals (const pcl::PolygonMesh::Ptr &mesh, pcl::PointCloud<pcl::Normal>::Ptr &normals) {

//	pcl::features::computeApproximateNormals(mesh->cloud, mesh->polygons, normals);
}

void centerCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
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

void fitToUnitCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
	float maxDist = 0.0;

	for (int i = 0; i < cloud->points.size(); ++i) {
		float dist = sqrt(pow(cloud->points[i].x, 2), pow(cloud->points[i].y, 2), pow(cloud->points[i].z, 2));

		if (dist > maxDist)
			maxDist = dist;
	}

	for (int i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x /= maxDist;
		cloud->points[i].y /= maxDist;
		cloud->points[i].z /= maxDist;
	}
}

void normalizeCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
	centerCloud(cloud);
	fitToUnitCloud(cloud);
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

void enterViewerLoop(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> normals)
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


