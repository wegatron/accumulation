#include "zsw_vtk_io.h"
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkFloatArray.h>
#include <vtkDoubleArray.h>
#include <vtkIntArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkPolyDataWriter.h>

int zsw::point_clouds2vtk_file(const std::string &vtk_file, const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &points_clouds)
{
	vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkFloatArray> index = vtkSmartPointer<vtkFloatArray>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	index->SetName("index");
	int cur_index = 0;
	for(const auto &point_cloud : points_clouds)
	{
		for(const auto &point : point_cloud->points)
		{
			auto id = points_data->InsertNextPoint(point.x, point.y, point.z);
			vertices->InsertNextCell(1);
			vertices->InsertCellPoint(id);
			index->InsertNextTuple1(cur_index);
		}
		++cur_index;
	}
	vtk_points_cloud->SetPoints(points_data);
	vtk_points_cloud->SetVerts(vertices);
	if(cur_index > 1) vtk_points_cloud->GetPointData()->SetScalars(index);

	// write out
	static vtkSmartPointer<vtkPolyDataWriter> poly_data_writer = vtkSmartPointer<vtkPolyDataWriter>::New();
	poly_data_writer->SetInputData(vtk_points_cloud);
	poly_data_writer->SetFileName(vtk_file.c_str());
	poly_data_writer->SetFileTypeToBinary();
	return poly_data_writer->Write();
}

int zsw::point_cloud2vtk_file(const std::string& vtk_file, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc,
	const std::vector<std::pair<std::string, std::vector<float>>> attributes)
{
	vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	for(const auto &point : pc->points)
	{
		const auto id = points_data->InsertNextPoint(point.x, point.y, point.z);
		vertices->InsertNextCell(1);
		vertices->InsertCellPoint(id);
	}
	vtk_points_cloud->SetPoints(points_data);
	vtk_points_cloud->SetVerts(vertices);

	for(const auto &attrib_data : attributes)
	{
		vtkSmartPointer<vtkFloatArray> tmp_attrib = vtkSmartPointer<vtkFloatArray>::New();
		tmp_attrib->SetName(attrib_data.first.c_str());
		assert(attrib_data.second.size() == pc->size());
		for(auto val : attrib_data.second)
		{
			tmp_attrib->InsertNextTuple1(val);
		}
		vtk_points_cloud->GetPointData()->AddArray(tmp_attrib);
	}

	// write out
	static vtkSmartPointer<vtkPolyDataWriter> poly_data_writer = vtkSmartPointer<vtkPolyDataWriter>::New();
	poly_data_writer->SetInputData(vtk_points_cloud);
	poly_data_writer->SetFileName(vtk_file.c_str());
	poly_data_writer->SetFileTypeToBinary();
	return poly_data_writer->Write();
}

int zsw::tensor2vtk_file(const std::string& vtk_file,
	const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& pts,
	const std::string &attribute_name,
	const std::vector<tensor_mat, Eigen::aligned_allocator<tensor_mat>>& tensor_data)
{
	vtkSmartPointer<vtkPoints> points_data = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPolyData> vtk_points_cloud = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkDoubleArray> ellipsoid = vtkSmartPointer<vtkDoubleArray>::New();
	ellipsoid->SetNumberOfComponents(9);
	ellipsoid->SetNumberOfTuples(pts.size());
	ellipsoid->SetName(attribute_name.c_str());
	auto len = pts.size();
	for(auto i=0; i<len; ++i)
	{
		const auto id = points_data->InsertNextPoint(pts[i].x(), pts[i].y(), pts[i].z());
		vertices->InsertNextCell(1);
		vertices->InsertCellPoint(id);
		ellipsoid->InsertTuple(i, tensor_data[i].data());
	}
	vtk_points_cloud->SetPoints(points_data);
	vtk_points_cloud->SetVerts(vertices);
	vtk_points_cloud->GetPointData()->AddArray(ellipsoid);
	// write out
	static vtkSmartPointer<vtkPolyDataWriter> poly_data_writer = vtkSmartPointer<vtkPolyDataWriter>::New();
	poly_data_writer->SetInputData(vtk_points_cloud);
	poly_data_writer->SetFileName(vtk_file.c_str());
	poly_data_writer->SetFileTypeToBinary();
	return poly_data_writer->Write();
}
