#ifndef ZSW_VTK_IO_H
#define ZSW_VTK_IO_H

#include <vector>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <io_common_export.h>

namespace zsw{
	IO_COMMON_EXPORT int point_clouds2vtk_file(const std::string &vtk_file, const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &points_clouds);

	IO_COMMON_EXPORT int point_cloud2vtk_file(const std::string &vtk_file, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc,
		const std::vector<std::pair<std::string, std::vector<float>>> attributes,
		const pcl::PointCloud<pcl::Normal>::Ptr normal = nullptr);

	typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> tensor_mat;

	/**
	 * \brief points with 3x3 tensor to vtk file.
	 * can be visualized as ellipsoid, refer to: http://www.parresianz.com/dem/ellipsoids-in-visit-and-paraview/
	 * \param vtk_file 
	 * \param pts
	 * \param attribute_name
	 * \param tensor_data 
	 * \return 0 if success
	 */
	IO_COMMON_EXPORT int tensor2vtk_file(const std::string &vtk_file,
		const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &pts,
		const std::string &attribute_name,
		const std::vector<tensor_mat, Eigen::aligned_allocator<tensor_mat>> &tensor_data);

	IO_COMMON_EXPORT int point_cloud2vtk_file2(const std::string &vtk_file, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc,
		const std::vector<std::tuple<std::string, int, std::vector<float>>> attributes,
		const pcl::PointCloud<pcl::Normal>::Ptr normal = nullptr);
}
#endif //ZSW_VTK_IO_H