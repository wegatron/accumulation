#include <string>
#include <boost/python.hpp>
#include <boost/regex.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "lasser3d_io.h"
#include "zsw_vtk_io.h"


void laser3d_reg2vtk(const std::string &ini_file,
	const std::string laser3d_reg_file, 
	const std::string &init_vtk,
	const std::string &reg_vtk)
{
	std::ifstream ini_ifs(ini_file);
	if(!ini_ifs) { std::cerr << "Unable to read file:" << ini_file << std::endl; return;}
	std::string tmp_str;
	boost::regex word_regex("\\s+");
	std::vector<std::string> fields;
	Eigen::Vector3f t=Eigen::Vector3f::Zero();
	float yaw = 0;
	float pitch = 0;
	float roll = 0;
	while(std::getline(ini_ifs, tmp_str))
	{
		fields.clear();
		boost::regex_split(std::back_inserter(fields), tmp_str, word_regex);
		if(fields.size() == 3)
		{
			if(fields[0] == "x_laser_in_odo")
			{
				t[0] = std::stof(fields[2]);
			} else if(fields[0] == "y_laser_in_odo")
			{
				t[1] = std::stof(fields[2]);
			} else if(fields[0] == "z_laser_in_odo")
			{
				t[2] = std::stof(fields[2]);
			} else if(fields[0] == "pitch_laser_to_odo")
			{
				pitch = std::stof(fields[2]) * M_PI / 180;
			} else if(fields[0] == "roll_laser_to_odo")
			{
				roll = std::stof(fields[2]) * M_PI / 180;
			} else if(fields[0] == "yaw_laser_to_odo")
			{
				yaw = std::stof(fields[2]) * M_PI / 180;
			}
		}
	}
	Eigen::Matrix4f laser2odo = (Eigen::Translation3f(t[0], t[1], t[2]) * Eigen::Affine3f(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()))).matrix();
	ini_ifs.close();

	auto reg_data = zsw_gz_common::load_laser3d_reg_data(laser3d_reg_file);
	Eigen::Matrix4f ini_rt = reg_data->init_rt * laser2odo;
	Eigen::Matrix4f reg_rt = reg_data->reg_rt * laser2odo;
	pcl::PointCloud<pcl::PointXYZ>::Ptr init_pc(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr reg_pc(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::transformPointCloud(*reg_data->pc_, *init_pc, ini_rt);
	pcl::transformPointCloud(*reg_data->pc_, *reg_pc, reg_rt);

	zsw::point_cloud2vtk_file(init_vtk, init_pc, {});
	zsw::point_cloud2vtk_file(reg_vtk, reg_pc, {});
}

void laser3d2pcd(const std::string laser3d_path, const std::string pcd_path)
{
	auto pc_ptr = zsw_gz_common::load_laser3d_frame(laser3d_path);
	pcl::io::savePCDFileBinary(pcd_path, *pc_ptr);
}


BOOST_PYTHON_MODULE(zsw_py_io) {
    using namespace boost::python;
    def("laser3d2pcd", laser3d2pcd),
	def("laser3d_reg2vtk", laser3d_reg2vtk);
}