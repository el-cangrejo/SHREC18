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

float g_curvaturePercentageThreshold{0.6};
bool g_writeCurvatures{true};
bool g_binaryCurvatures{true};
using namespace Eigen;

bool init(igl::viewer::Viewer& viewer) {
	viewer.ngui->addButton("Load Mesh", [&]() {
		std::string fname = igl::file_dialog_open();

		if (fname.length() == 0) return;

		std::string mesh_file_name_string = std::string(fname);
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
			igl::read_triangle_mesh(mesh_file_name_string, V, F);
			viewer.data.clear();
			viewer.data.set_mesh(V, F);
			std::cout << "Ply file" << std::endl;
		}
		viewer.data.compute_normals();
		viewer.data.uniform_colors(
		    Eigen::Vector3d(51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0),
		    Eigen::Vector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0),
		    Eigen::Vector3d(255.0 / 255.0, 235.0 / 255.0,
				    80.0 / 255.0));
		if (viewer.data.V_uv.rows() == 0) {
			viewer.data.grid_texture();
		}
		viewer.core.align_camera_center(viewer.data.V, viewer.data.F);
	});
	viewer.ngui->addVariable("Write Curvatures", g_writeCurvatures);
	viewer.ngui->addVariable("Binary Curvatures", g_binaryCurvatures);
	viewer.ngui->addVariable("Curvature threshold",
				 g_curvaturePercentageThreshold);
	viewer.screen->performLayout();
	return false;
}

VectorXd computeCurvature(const MatrixXd& V, const MatrixXi& F) {
	std::cout << "Computing mean curvature" << std::endl;
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

	if (g_binaryCurvatures) {
		auto maxValue = H.col(0).maxCoeff();
		auto minValue = H.col(0).minCoeff();
		// std::cout << "maxvalue" << maxValue << std::endl;
		// std::cout << "minValue" << minValue << std::endl;
		double threshold =
		    g_curvaturePercentageThreshold * (maxValue - minValue) +
		    minValue;
		// std::cout << "threshold" << threshold << std::endl;
		for (int row = 0; row < H.rows(); row++) {
			if (H(row, 0) < threshold)
				H(row, 0) = 0;
			else
				H(row, 0) = 1;
		}
	}
	return H;
}

void writeToFile(const VectorXd& v, std::string filename) {
	std::ofstream out(filename + ".txt");
	for (int row = 0; row < v.rows(); row++) {
		out << v(row, 0) << "\n";
	}
	out.close();
}

bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int mod) {
	if (key == 'M') {
		VectorXd H = computeCurvature(viewer.data.V, viewer.data.F);
		if (g_writeCurvatures) {  // write to file
			writeToFile(H, "curvatures");
		}
		MatrixXd C;
		// Compute pseudocolor
		igl::parula(H, true, C);
		viewer.data.set_colors(C);
		return true;
	} else {
		return false;
	}
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
	viewer.core.show_lines = false;
	VectorXd H;

	viewer.data.set_mesh(V, F);

	viewer.callback_key_down = &key_down;
	viewer.callback_init = &init;
	viewer.launch();
}
