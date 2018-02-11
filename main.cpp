#include <igl/avg_edge_length.h>
#include <igl/cotmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/invert_diag.h>
#include <igl/massmatrix.h>
#include <igl/parula.h>
#include <igl/per_corner_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/principal_curvature.h>
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>
#include <nanogui/formhelper.h>
#include <nanogui/screen.h>

#include <iostream>

#include <pcl/features/boundary.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>

#include "comparison.h"
#include "helpers.hpp"
#include "viewer.hpp"

void load_mesh_from_file(std::string mesh_file_name) {}
void open_dialog_load_mesh() {
}
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
	viewer.callback_init = [&](igl::viewer::Viewer& viewer) {

		// Add new group
		viewer.ngui->addGroup("Mesh IO");
		// Add a button
		viewer.ngui->addButton("Load Mesh", [&]() {
	std::string fname = igl::file_dialog_open();

	if (fname.length() == 0) return;

			std::string mesh_file_name_string =
			    std::string(fname);
			size_t last_dot = mesh_file_name_string.rfind('.');
			if (last_dot == std::string::npos) {
				printf("Error: No file extension found in %s\n",
				       mesh_file_name_string);
			}
			std::string extension =
			    mesh_file_name_string.substr(last_dot + 1);
			if (extension == "ply" || extension == "PLY") {
				Eigen::MatrixXd V;
				Eigen::MatrixXi F;
				std::cout << "Ply file" << std::endl;
			}
				std::cout<<"load mesh"<<std::endl;
		});

		// call to generate menu
		viewer.screen->performLayout();
				return false;
	};
	viewer.launch();
}
