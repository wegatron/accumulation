#include "zsw_vtk_io.h"

int main(int argc, char* argv[])
{
	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts{ Eigen::Vector3d(0,0,0) };
	zsw::tensor_mat mat;
	mat << 1, 0, 0,
		0, 2, 0,
		0, 0, 4;
	std::vector<zsw::tensor_mat, Eigen::aligned_allocator<zsw::tensor_mat>> tensor_data{ mat };

	zsw::tensor2vtk_file("F:/tmp_zsw/tmp.vtk",
		pts,
		"demo_ellipsoid",
		tensor_data);
	return 0;
}
