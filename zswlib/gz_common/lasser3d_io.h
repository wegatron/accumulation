#ifndef LASSER3D_IO_H
#define LASSER3D_IO_H

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace zsw_gz_common
{

	/**
	 * \brief load laser frame with pose, from 'graph_*.txt'
	 * \param graph_file file path of `graph_*.txt`
	 * \param laser_pcs structured laser frames
	 * \param transforms each laser frame's transform
	 * \return 0 if success
	 */
	int load_graph_nodes(const std::string &graph_file,
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &laser_pcs,
		std::vector<Eigen::Affine3f> &transforms);

	/**
	 * \brief load structured laser frame from file
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr load_laser3d_frame(const std::string &laser3d_file);

}
#endif //LASSER3D_IO_H