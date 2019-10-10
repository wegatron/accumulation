#include "zsw_vtk_io.h"

#include <boost/graph/adjacency_list.hpp>
#include "lasser3d_io.h"

int main(int argc, char* argv[])
{
	// boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS> g;
	// auto v0 = boost::add_vertex(g);
	// auto v1 = boost::add_vertex(g);
	// auto v2 = boost::add_vertex(g);
	//
	// auto vitr = boost::vertices(g);
	// auto vv2 = *(vitr.first+2);
	// std::cout << vv2 << std::endl;
	// std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts{ Eigen::Vector3d(0,0,0) };
	// zsw::tensor_mat mat;
	// mat << 1, 0, 0,
	// 	0, 2, 0,
	// 	0, 0, 4;
	// std::vector<zsw::tensor_mat, Eigen::aligned_allocator<zsw::tensor_mat>> tensor_data{ mat };
	//
	// zsw::tensor2vtk_file("F:/tmp_zsw/tmp.vtk",
	// 	pts,
	// 	"demo_ellipsoid",
	// 	tensor_data);
	auto ret = zsw_gz_common::load_laser3d_reg_data("F:/data/gz_3dloc_problem/2019.8.30_kaufaqu/log/OffData-20190829T064037/debug/P_0_000023_4295.00.bin");
	zsw::point_cloud2vtk_file("f:/data/tmp/debug.vtk", ret->pc_, {});
	return 0;
}
