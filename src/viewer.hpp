#ifndef VIEWER_HPP
#define VIEWER_HPP

#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
void enterViewerLoop(pcl::PointCloud<pcl::PointXYZ> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals,
		     pcl::PointXYZ sphereCenter,
		     std::vector<pcl::PointXYZ> blueSpheres,
		     float sphereRadius1, float sphereRadius2) {
	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "sample cloud");
	viewer.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
	    cloud.makeShared(), normals.makeShared(), 10, 0.05, "normals");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();

	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}

void enterViewerLoopMesh(pcl::PolygonMesh &mesh,
			 pcl::PointCloud<pcl::Normal> &normals,
			 pcl::PointXYZ sphereCenter, float sphereRadius1,
			 float sphereRadius2) {
	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromPCLPointCloud2(mesh.cloud, cloud);
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	viewer.addPolygonMesh(mesh, "sample cloud");
	viewer.addSphere(sphereCenter, sphereRadius1, 0.0, 1.0, 0.0, "sphere");
	viewer.addSphere(sphereCenter, sphereRadius2, 1.0, 0.0, 0.0, "sphere1");
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}

void enterViewerLoop(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals,
		     std::vector<int> nodes, float sphereRadius) {
	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud.makeShared(),
					       "sample cloud");
	viewer.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(
	//	    cloud.makeShared(), normals.makeShared(), 10, 0.05,
	//"normals");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	viewer.addSphere(cloud[nodes[0]], sphereRadius, 0.0, 1.0, 0.0,
			 "sphere");
	for (int i = 1; i < nodes.size(); i++) {
		viewer.addSphere(cloud.points[nodes[i]], sphereRadius,
				 0.0, 0.0, 1.0, "sphere" + std::to_string(i));
	}
	// for (int i = 1; i < pointIndices.size(); i++)
	// viewer.addSphere(cloud[pointIndices[i]], sphereRadius, 1.0, 0.0, 1.0,
	// 		 "closest sphere" + std::to_string(i));
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}

void enterViewerLoop(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals,
		     std::vector<std::vector<int>> graphs, float sphereRadius) {
	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud.makeShared(),
					       "sample cloud");
	viewer.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(
	//	    cloud.makeShared(), normals.makeShared(), 10, 0.05,
	//"normals");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	for (int i = 0; i < graphs.size(); i++) {
		float graph_color = (float)i / graphs.size();
		viewer.addSphere(cloud[graphs[i][0]], sphereRadius, 0.0, 1.0, 0.0,
				 "first_sphere" + std::to_string(i));
		for (int j = 1; j < graphs[i].size(); ++j) 
			viewer.addSphere(cloud.points[graphs[i][j]], sphereRadius, 0.0, 0.0, graph_color, "sphere" + std::to_string(i) + "_" + std::to_string(j));
	}
	// for (int i = 1; i < pointIndices.size(); i++)
	// viewer.addSphere(cloud[pointIndices[i]], sphereRadius, 1.0, 0.0, 1.0,
	// 		 "closest sphere" + std::to_string(i));
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}
void enterViewerLoop(pcl::PolygonMesh &mesh,
				 pcl::PointCloud<pcl::PointXYZ> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals,
		     std::vector<std::vector<int>> graphs, float sphereRadius) {

	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();

	viewer.addPolygonMesh<pcl::PointXYZ>(cloud.makeShared(), mesh.polygons, "sample cloud");

	for (int i = 0; i < graphs.size(); i++) {
		float graph_color = (float)i / graphs.size();
		viewer.addSphere(cloud[graphs[i][0]], sphereRadius, 0.0, 1.0, 0.0,
				 "first_sphere" + std::to_string(i));
		for (int j = 1; j < graphs[i].size(); ++j) 
			viewer.addSphere(cloud.points[graphs[i][j]], sphereRadius, 0.0, 0.0, graph_color, "sphere" + std::to_string(i) + "_" + std::to_string(j));
	}
	// for (int i = 1; i < pointIndices.size(); i++)
	// viewer.addSphere(cloud[pointIndices[i]], sphereRadius, 1.0, 0.0, 1.0,
	// 		 "closest sphere" + std::to_string(i));
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}

void enterViewerLoopMesh(pcl::PolygonMesh &mesh,
			 pcl::PointCloud<pcl::PointXYZRGB> &cloudrgb,
			 pcl::PointCloud<pcl::Normal> &normals) {
	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	// viewer.addPolygonMesh(cloudrgb.makeShared(), mesh.polygons, "sample
	// cloud");
	// viewer.addPointCloud<pcl::PointXYZRGB>(cloudrgb.makeShared(),
	//				       "rgb cloud");
	//	viewer.setPointCloudRenderingProperties(
	//	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample
	// cloud");
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromPCLPointCloud2(mesh.cloud, cloud);
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
	    cloud.makeShared(), normals.makeShared(), 1, 0.05, "normals");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}

#endif  // VIEWER_HPP
