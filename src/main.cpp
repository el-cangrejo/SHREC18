#include <iostream>
#include <string>
#include "comparison.h"
#include "helpers.hpp"
#include "viewer.hpp"

int main(int argc, char **argv) {
	std::cout << "Hello SHREC2018!\n";

	//std::string queryModel = "shrec18_recognition/Dataset/" +
	//			 std::to_string(atoi(argv[1])) + ".ply";
	std::string queryModel = "shrec18_recognition/Queries/" +
	std::to_string(atoi(argv[1])) + ".ply";
	pcl::PointCloud<pcl::PointXYZ> cloud_q;
	pcl::PointCloud<pcl::Normal> normals_q;
	pcl::PolygonMesh mesh_q;

	// std::string targetModel = "shrec18_recognition/Dataset/6.ply";
	// pcl::PointCloud<pcl::PointXYZ> targetCloud;
	// pcl::PointCloud<pcl::Normal> targetNormals;
	// pcl::PolygonMesh targetMesh;

	if (pcl::io::loadPLYFile(queryModel, mesh_q) == -1) {
		return -1;
	}
	// if (pcl::io::loadPLYFile(targetModel, targetMesh) == -1) {
	//	return -1;
	//}

	int idx = atoi(argv[2]);
	float inner_radius = atof(argv[3]);
	float outer_radius = atof(argv[4]);

	pcl::fromPCLPointCloud2(mesh_q.cloud, cloud_q);

	std::cout << "Query model : " << cloud_q.points.size() << "\n";

	computeNormals(mesh_q, cloud_q, normals_q);

	CloudWithNormals cloud_qWithNormals(cloud_q, normals_q);
	auto features_q =
	    computeFeaturesSHOT352(cloud_qWithNormals, inner_radius);

	// pcl::fromPCLPointCloud2(targetMesh.cloud, targetCloud);
	// computeNormals(targetMesh, targetCloud, targetNormals);
	// CloudWithNormals targetCloudWithNormals(targetCloud, targetNormals);
	// auto targetFeatures =
	//    computeFeatures_SHOT352(targetCloudWithNormals,
	//    searchTargetRadius);

	int featureSize = 352;
	std::vector<float> distances = selfFeatureDistance(features_q, idx);
	pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
	createRGBCloud(cloud_q, distances, cloud_rgb);
	centerCloud<pcl::PointXYZRGB>(cloud_rgb);

	// auto pfh_inner = computePointFeatures_PFH(cloud_qWithNormals, idx,
	// inner_radius);
	// auto pfh_outer = computePointFeatures_PFH(cloud_qWithNormals, idx,
	// outer_radius);
	// auto h = histogramDifference(pfh_inner.points[0].histogram,
	// pfh_outer.points[0].histogram, 125);
	std::vector<int> idx_t;
	idx_t.push_back(idx);
	float r = inner_radius;

	for (int i = 1; i < 2; ++i) {
		int idx_tmp = 0;
		float r_tmp = 0;

		std::cout << "Radius " << r << "\n";
		std::cout << "Out Radius " << outer_radius << "\n";
		std::cout << "Point " << idx_t[i-1] << "\n";
		minDistPoint(cloud_q, features_q, r, outer_radius, idx_t[i - 1], idx_tmp, r_tmp);
		std::cout << "Point found " << idx_tmp << "\n";

		r = inner_radius + r_tmp;
		idx_t.push_back(idx_tmp);
	}

	enterViewerLoop(cloud_rgb, normals_q, idx_t, inner_radius);
}
