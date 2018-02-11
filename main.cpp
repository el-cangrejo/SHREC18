#include <igl/avg_edge_length.h>
#include <igl/cotmatrix.h>
#include <igl/invert_diag.h>
#include <igl/massmatrix.h>
#include <igl/parula.h>
#include <igl/per_corner_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/principal_curvature.h>
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>
#include <iostream>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/principal_curvatures.h>

#include "helpers.hpp"
#include "comparison.h"
#include "viewer.hpp"

void computeMeanCurvature(int a) { using namespace Eigen; }

int main(int argc, char* argv[]) {
	using namespace Eigen;
	std::string filepath = "shrec18_recognition/Queries/";
	std::string filename = filepath + "1.ply";
	if (argc > 1) {
		filename = argv[1];
	}
	// Load a mesh in OFF format
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	igl::read_triangle_mesh(filename, V, F);
	std::cout << "Number of vertices: " << V.rows() << std::endl;

	igl::viewer::Viewer viewer;
	VectorXd H;
	// computeMeanCurvature(V, F, H, viewer);

	viewer.data.set_mesh(V, F);


	viewer.callback_key_down = [](igl::viewer::Viewer& viewer,
				      unsigned char key, int mod) -> bool {
		if (key == 'M') {
			std::cout << "Computing mean curvature" << std::endl;
			const MatrixXd& V = viewer.data.V;
			const MatrixXi& F = viewer.data.F;

			// Alternative discrete mean curvature
			MatrixXd HN;
			SparseMatrix<double> L, M, Minv;
			igl::cotmatrix(V, F, L);
			igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_VORONOI, M);
			igl::invert_diag(M, Minv);
			// Laplace-Beltrami of position
			HN = -Minv * (L * V);
			// Extract magnitude as mean curvature
			VectorXd H = HN.rowwise().norm();
			// fitting
			MatrixXd PD1, PD2;
			VectorXd PV1, PV2;
			igl::principal_curvature(V, F, PD1, PD2, PV1, PV2);
			// mean curvature
			H = 0.5 * (PV1 + PV2);

			double threshold = -0.6;
			for (int row = 0; row < H.rows(); row++) {
				for (int col = 0; col < H.cols(); col++) {
					if (H(row, col) < threshold)
						H(row, col) = 0;
					else
						H(row, col) = 1;
				}
			}
			// Compute pseudocolor
			MatrixXd C;
			igl::parula(H, true, C);
			viewer.data.set_colors(C);
			return true;
		} else {
			return false;
		}
	};

	viewer.launch();
}
