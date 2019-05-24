#include "lasser3d_io.h"

#include <boost/regex.hpp>
#include <boost/filesystem/path.hpp>
#include <fstream>

namespace zsw_gz_common
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr load_laser3d_frame(const std::string& laser3d_file)
	{
		int beam_size = 0;
		int seg_size = 0;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pts_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		std::ifstream ifs(laser3d_file, std::ifstream::binary);
		if (!ifs) return pts_cloud;
		boost::regex word_regex("\\s+");
		while (true)
		{
			std::string tmp_str;
			std::getline(ifs, tmp_str);
			std::vector<std::string> fields;
			regex_split(std::back_inserter(fields), tmp_str, word_regex);
			if (fields[0] == "#segment_beam_size")
			{
				beam_size = std::stoi(fields[1]);
			}
			else if (fields[0] == "#segment_size")
			{
				seg_size = std::stoi(fields[1]);
			}
			else if (fields[0] == "#data_binary")
				break;
		}
		int seg_index = -1;
		pcl::PointXYZ tmp_pt;
		char buff[10];
		while (true)
		{
			if (++seg_index > seg_size) break;
			ifs.read(buff, sizeof(float)); // read time
			for (int i = 0; i < beam_size; ++i)
			{
				// pts_cloud->insert()
				ifs.read(reinterpret_cast<char*>(&tmp_pt.x), sizeof(float));
				ifs.read(reinterpret_cast<char*>(&tmp_pt.y), sizeof(float));
				ifs.read(reinterpret_cast<char*>(&tmp_pt.z), sizeof(float));
				ifs.read(&buff[0], 1); // is valid
				ifs.read(&buff[1], 1); // remission
				// if (buff[0] == 0) continue;
				double squared_norm = tmp_pt.x * tmp_pt.x + tmp_pt.y * tmp_pt.y + tmp_pt.z * tmp_pt.z;
				if (squared_norm < 0.125) continue;
				pts_cloud->push_back(tmp_pt);
			}
		}
		return pts_cloud;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr load_pts_binary(const std::string& pc_file)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pts_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		std::ifstream cloud_file;
		cloud_file.open(pc_file, std::ios::binary);
		if (cloud_file.is_open() == false) {
			std::cout << "open cloud file fail!" << std::endl;
			return nullptr;
		}

		std::string point_str;
		bool if_get_binary_flag = false;
		while (getline(cloud_file, point_str)) {
			if (point_str == "#cloud_data_binary") {
				if_get_binary_flag = true;
				break;
			}
		}

		if (if_get_binary_flag == false) {
			cloud_file.close();
			std::cout << "read cloud file binary " << pc_file << " fail!" << std::endl;
			return nullptr;
		}

		while (true) {
			float x, y, z;
			unsigned char rssi;
			cloud_file.read((char*)(&x), sizeof(float));
			if (cloud_file.gcount() < sizeof(float)) {
				break;
			}
			cloud_file.read((char*)(&y), sizeof(float));
			if (cloud_file.gcount() < sizeof(float)) {
				break;
			}
			cloud_file.read((char*)(&z), sizeof(float));
			if (cloud_file.gcount() < sizeof(float)) {
				break;
			}
			cloud_file.read((char*)(&rssi), sizeof(unsigned char));
			if (cloud_file.gcount() < sizeof(unsigned char)) {
				break;
			}
			pcl::PointXYZ point;
			point.x = x;  point.y = y;  point.z = z;
			pts_cloud->points.push_back(point);
		}

		cloud_file.close();
		std::cout << "read cloud file binary " << pc_file << " success! point size " << pts_cloud->points.size() << std::endl;
		return pts_cloud;
	}

	void write_pts_binary(const std::string& pc_file, pcl::PointCloud<pcl::PointXYZ>& pc)
	{
		std::ofstream ofs(pc_file, std::ofstream::binary);
		ofs << "#cloud_data_binary" << std::endl;
		char rssi = 0;
		for(const auto pt : pc.points)
		{
			ofs.write(reinterpret_cast<const char*>(&pt.x), sizeof(pt.x));
			ofs.write(reinterpret_cast<const char*>(&pt.y), sizeof(pt.y));
			ofs.write(reinterpret_cast<const char*>(&pt.z), sizeof(pt.z));
			ofs.write(&rssi, sizeof(rssi));
		}
		ofs.close();
	}


	int load_graph_nodes(const std::string& graph_file,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& laser_pcs,
		std::vector<Eigen::Affine3f>& transforms)
	{
		//// parse graph file
		std::ifstream graph_ifs(graph_file);
		if (!graph_ifs) { std::cerr << "error unable to open file:" << graph_file << std::endl; exit(__LINE__); }
		// get graph file dir
		boost::filesystem::path path(graph_file);
		const std::string laser3d_dir = path.parent_path().string();
		const boost::regex word_regex("\\s+");
		std::string tmp_str;
		Eigen::Affine3f r = Eigen::Affine3f::Identity();
		Eigen::Affine3f t = Eigen::Affine3f::Identity();
		// Eigen::Affine3f laser3d_2_odo = Eigen::Affine3f::Identity();
		while (!graph_ifs.eof())
		{
			std::vector<std::string> fields;
			std::getline(graph_ifs, tmp_str);
			regex_split(std::back_inserter(fields), tmp_str, word_regex);
			if (fields.size() < 2) continue;
			else if (fields[0] == "pose3d")
			{
				Eigen::Affine3f tt;
				tt = Eigen::Translation3f(Eigen::Vector3f(0, 0, 0));
				t = Eigen::Translation3f(Eigen::Vector3f(stof(fields[1]), stof(fields[2]), stof(fields[3])));
				const float radian_z = std::stof(fields[4]) / 180.0f * 3.1415926; // yaw
				const float radian_y = std::stof(fields[5]) / 180.0f * 3.1415926; // pitch
				const float radian_x = std::stof(fields[6]) / 180.0f * 3.1415926; // roll
				r = Eigen::Affine3f(Eigen::AngleAxisf(radian_z, Eigen::Vector3f::UnitZ()) *
					Eigen::AngleAxisf(radian_y, Eigen::Vector3f::UnitY()) *
					Eigen::AngleAxisf(radian_x, Eigen::Vector3f::UnitX()));
			}
			else if (fields[0] == "laser3d_file_name")
			{
				auto laser3d_file = laser3d_dir + "/" + fields[1];
				pcl::PointCloud<pcl::PointXYZ>::Ptr laser3d_frame = load_laser3d_frame(laser3d_file);
				laser_pcs.push_back(laser3d_frame);
				transforms.push_back(t * r);
			}
		}
		return 0;
	}
}
