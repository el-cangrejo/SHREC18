#include <utils.hpp>
#include <pcl/conversions.h>

void computeNormals(const pcl::PolygonMesh &mesh,
		    pcl::PointCloud<pcl::PointXYZ> &cloud,
		    pcl::PointCloud<pcl::Normal> &normals) {
	//pcl::features::computeApproximateNormals_(cloud, mesh.polygons,
	//					 normals);
	computeApproximateNormals_(cloud, mesh.polygons,
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

void enterViewerLoopMesh(pcl::PolygonMesh &mesh,
		     pcl::PointCloud<pcl::Normal> &normals) {
	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);
//	viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "sample cloud");
	viewer.addPolygonMesh(mesh, "sample cloud");
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromPCLPointCloud2(mesh.cloud, cloud);
//	viewer.setPointCloudRenderingProperties(
//	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
	    cloud.makeShared(), normals.makeShared(), 10, 0.05, "normals");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}

void computeApproximateNormals_(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::vector<pcl::Vertices>& polygons, pcl::PointCloud<pcl::Normal>& normals)
{
  int nr_points = static_cast<int>(cloud.points.size());
  int nr_polygons = static_cast<int>(polygons.size());

  normals.header = cloud.header;
  normals.width = cloud.width;
  normals.height = cloud.height;
  normals.points.resize(nr_points);

  for ( int i = 0; i < nr_points; ++i )
    normals.points[i].getNormalVector3fMap() = Eigen::Vector3f::Zero();

  // NOTE: for efficiency the weight is computed implicitly by using the
  // cross product, this causes inaccurate normals for meshes containing
  // non-triangle polygons (quads or other types)
  for ( int i = 0; i < nr_polygons; ++i )
  {
    const int nr_points_polygon = (int)polygons[i].vertices.size();
    if (nr_points_polygon < 3) continue;

    // compute normal for triangle
    Eigen::Vector3f vec_a_b = cloud.points[polygons[i].vertices[0]].getVector3fMap() - cloud.points[polygons[i].vertices[1]].getVector3fMap();
    Eigen::Vector3f vec_a_c = cloud.points[polygons[i].vertices[0]].getVector3fMap() - cloud.points[polygons[i].vertices[2]].getVector3fMap();
    Eigen::Vector3f normal = vec_a_b.cross(vec_a_c);
//    pcl::flipNormalTowardsViewpoint(cloud.points[polygons[i].vertices[0]], 0.0f, 0.0f, 0.0f, normal(0), normal(1), normal(2));

    // add normal to all points in polygon
    for ( int j = 0; j < nr_points_polygon; ++j )
      normals.points[polygons[i].vertices[j]].getNormalVector3fMap() += normal;
  }

  for ( int i = 0; i < nr_points; ++i )
  {
    normals.points[i].getNormalVector3fMap().normalize();
//    pcl::flipNormalTowardsViewpoint(cloud.points[i], 0.0f, 0.0f, 0.0f, normals.points[i].normal_x, normals.points[i].normal_y, normals.points[i].normal_z);
  }
}
