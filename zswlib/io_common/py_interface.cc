#include <string>
#include <boost/python.hpp>
#include <pcl/io/pcd_io.h>
#include "lasser3d_io.h"


void laser3d2pcd(const std::string laser3d_path, const std::string pcd_path)
{
	auto pc_ptr = zsw_gz_common::load_laser3d_frame(laser3d_path);
	pcl::io::savePCDFileBinary(pcd_path, *pc_ptr);
}


BOOST_PYTHON_MODULE(zsw_py_io) {
    using namespace boost::python;
    def("laser3d2pcd", laser3d2pcd);
}