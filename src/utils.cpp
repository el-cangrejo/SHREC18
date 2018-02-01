#include <utils.hpp>
#include <pcl/conversions.h>

void computeNormals(const pcl::PolygonMesh &mesh,
		    pcl::PointCloud<pcl::PointXYZ> &cloud,
		    pcl::PointCloud<pcl::Normal> &normals) {
	pcl::fromPCLPointCloud2(mesh.cloud, cloud);
	pcl::features::computeApproximateNormals(cloud, mesh.polygons,
						 normals);
}

void centerCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) {
	pcl::CentroidPoint<pcl::PointXYZ> centroid;
	for (int i = 0; i < cloud.points.size(); ++i) {
		centroid.add(cloud.points[i]);
	}
	pcl::PointXYZ c;
	centroid.get(c);

	std::cout << "Centroid: " << c << "\n";
	for (int i = 0; i < cloud.points.size(); ++i) {
		cloud.points[i].x = cloud.points[i].x - c.x;
		cloud.points[i].y = cloud.points[i].y - c.y;
		cloud.points[i].z = cloud.points[i].z - c.z;
	}
}

void fitToUnitCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) {
	float maxDist = 0.0;

	for (int i = 0; i < cloud.points.size(); ++i) {
		float dist = sqrt(pow(cloud.points[i].x, 2) +
				  pow(cloud.points[i].y, 2) +
				  pow(cloud.points[i].z, 2));

		if (dist > maxDist) maxDist = dist;
	}

	for (int i = 0; i < cloud.points.size(); ++i) {
		cloud.points[i].x /= maxDist;
		cloud.points[i].y /= maxDist;
		cloud.points[i].z /= maxDist;
	}
}

void normalizeCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) {
	centerCloud(cloud);
	fitToUnitCloud(cloud);
}

void enterViewerLoop(pcl::PointCloud<pcl::PointXYZ> &cloud,
		     pcl::PointCloud<pcl::Normal> &normals) {
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
