#ifndef ZSW_IO_H
#define ZSW_IO_H

#include <string>
#include <map>
#include <vector>

#include <Eigen/Dense>

namespace zsw{

  namespace obj{
  /**
   *!!!Be careful, the Eigen Matrix must store in column-major(default storage major).
   */
  int readTexture(const std::string &path, std::map<int,int> &texture_map,
                  Eigen::Matrix<double, 2, -1> &texture_points);
  /**
   *!!!Be careful, the Eigen Matrix must store in column-major(default storage major).
   */
  int readNormal(const std::string &path, std::map<int,int> &normal_map,
                 Eigen::Matrix<double, 3, -1> &normals);
  /**
   * read a triangle mesh in obj.
   * !!!Be careful, the Eigen Matrix must store in column-major(default storage major).
   */
  int readObjBasic(const std::string &path, Eigen::Matrix<double,3,-1> &vertexes,
                   Eigen::Matrix<int,3,-1> &faces);

  int writeObjMesh(const std::string &path, const Eigen::Matrix<double, 3, -1> &vertexes,
                   const Eigen::Matrix<int, 3, -1> &faces,
                   const Eigen::Matrix<double, 3, -1> *normals=NULL,
                   const std::map<int,int> *normal_map=NULL,
                   const std::string *mtl_file=NULL, const std::string *mtl_name=NULL,
                   const Eigen::Matrix<double, 2, -1> *texture_points=NULL,
                   const std::map<int,int> *texture_map=NULL);
  }

}

#endif /* ZSW_IO_H */
