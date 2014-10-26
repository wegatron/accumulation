#include "zsw_io.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

int zsw::obj::readTexture(const string &path, std::map<int,int> &texture_map, Eigen::Matrix<double, 2, -1> &texture_points)
{
  ifstream ifs(path);
  if(!ifs) {
    cerr << "can not open file: " << path << endl;
    return -1;
  }

  vector<double> texture_v;
  double tmp_double[2];
  int tmp_int[2];
  char tmp_ch;
  string str;
  while(!ifs.eof()) {
    ifs >> str;
    if(str == "vt") {
      ifs >> tmp_double[0] >> tmp_double[1];
      texture_v.push_back(tmp_double[0]);
      texture_v.push_back(tmp_double[1]);
    }

    if(str == "f") {
      for (int i=0; i<3; ++i) {
        ifs >> str;
        if (str.find("/")==string::npos || str.find("//")!=string::npos) {
          cerr << "[WARNING] texture map is empty on face." << endl;
        } else {
          // @todo fix bugs
          stringstream ss(str);
          ss >> tmp_int[0] >> tmp_ch >> tmp_int[1];
          texture_map[tmp_int[0]] = tmp_int[1];
        }
      }
    }
  }

  ifs.close();
  assert(texture_v.size()%2==0);
  texture_points.resize(2, texture_v.size()/2);
  std::copy(texture_v.begin(), texture_v.end(), texture_points.data());
  return 0;
}

int zsw::obj::readNormal(const std::string &path, std::map<int,int> &normal_map,
                    Eigen::Matrix<double, 3, -1> &normals)
{
  ifstream ifs(path);
  if(!ifs) {
    cerr << "can not open file: " << path << endl;
    return -1;
  }

  string str;
  double tmp[3];
  int tmp_int[2];
  vector<double> normal_v;
  while(!ifs.eof()) {
    ifs >> str;
    if(str == "vn") {
      ifs >> tmp[0] >> tmp[1] >> tmp[2];
      normal_v.push_back(tmp[0]);
      normal_v.push_back(tmp[1]);
      normal_v.push_back(tmp[2]);
    }

    if(str=="f") {
      for(int i=0; i<3; ++i) {
        ifs >> str;
        if(str.find("/") == str.rfind("/")) {
          cerr << "[WARNING] normal map is empty on face." << endl;
        } else {
          stringstream ss(str);
          ss >> tmp_int[0];
          ss.str(str.substr(str.rfind("/")+1)); ss.clear();
          ss >> tmp_int[1];
          normal_map[tmp_int[0]] = tmp_int[1];
        }
      }
    }
  }

  ifs.close();
  assert(normal_v.size()%3==0);
  normals.resize(3, normal_v.size()/3);
  std::copy(normal_v.begin(), normal_v.end(), normals.data());
  return 0;
}

int zsw::obj::readObjBasic(const std::string &path, Eigen::Matrix<double,3,-1> &vertexes,
                      Eigen::Matrix<int,3,-1> &faces)
{
  ifstream ifs(path);
  if(!ifs) {
    cerr << "can not openfile: " << path << endl;
    return -1;
  }

  string str;
  double tmp_double[3];
  int tmp_int;

  vector<double> vertex_v;
  vector<int> face_v;

  while(!ifs.eof()) {
    ifs >> str;
    if(str == "v") {
      ifs >> tmp_double[0] >> tmp_double[1] >> tmp_double[2];
      vertex_v.push_back(tmp_double[0]);
      vertex_v.push_back(tmp_double[1]);
      vertex_v.push_back(tmp_double[2]);
    }

    if(str=="f") {
      for (int i=0; i<3; ++i) {
        ifs >> str;
        stringstream ss(str);
        ss >> tmp_int;
        face_v.push_back(tmp_int);
      }
    }
  }

  ifs.close();
  assert(vertex_v.size()%3==0); assert(face_v.size()%3==0);
  vertexes.resize(3,vertex_v.size()/3);
  faces.resize(3,face_v.size()/3);

  std::copy(vertex_v.begin(), vertex_v.end(), vertexes.data());
  std::copy(face_v.begin(), face_v.end(), faces.data());
  return 0;
}

int zsw::obj::writeObjMesh(const string &path, const Eigen::Matrix<double,3,-1> &vertexes,
                      const Eigen::Matrix<int,3,-1> &faces,
                      const Eigen::Matrix<double,3,-1> *normals,
                      const map<int,int> *normal_map,
                      const string *mtl_file, const string *mtl_name,
                      const Eigen::Matrix<double,2,-1> *texture_points,
                      const map<int,int> *texture_map)
{
  ofstream ofs(path);
  if(!ofs) {
    cerr << "can open file: " << path << " for write!" << endl;
    return -1;
  }

  if(mtl_file!=NULL) {
    ofs << "mtllib " << *mtl_file << endl;
  }

  for (int i=0; i<vertexes.cols(); ++i) {
    ofs << "v " << vertexes(0,i) << " " << vertexes(1,i) << " " << vertexes(2,i) << endl;
  }

  if(normals!=NULL) {
    for (int i=0; i<normals->cols(); ++i) {
      ofs << "vn " << normals->block<3,1>(0,i).transpose() << endl;
    }
  }

  if (texture_points!=NULL) {
    for(int i=0; i<texture_points->cols(); ++i) {
      ofs << "vt " << texture_points->block<2,1>(0,i).transpose() << endl;
    }
  }

  if(mtl_name!=NULL) {
    ofs << "usemtl " << *mtl_name << endl;
  }
  if (normal_map!=NULL && texture_map!=NULL) {
    for (int i=0; i<faces.cols(); ++i) {
      ofs << "f "
          << faces(0,i) << "/" << texture_map->find(faces(0,i))->second << "/" << normal_map->find(faces(0,i))->second<< " "
          << faces(1,i) << "/" << texture_map->find(faces(1,i))->second << "/" << normal_map->find(faces(1,i))->second<< " "
          << faces(2,i) << "/" << texture_map->find(faces(2,i))->second << "/" << normal_map->find(faces(2,i))->second<< endl;
    }
  } else if(normal_map!=NULL) {
    for (int i=0; i<faces.cols(); ++i) {
      ofs << "f "
          << faces(0,i) << "//" << normal_map->find(faces(0,i))->second << " "
          << faces(1,i) << "//" << normal_map->find(faces(1,i))->second << " "
          << faces(2,i) << "//" << normal_map->find(faces(2,i))->second << endl;

    }
  }else if(texture_map!=NULL) {
    for (int i=0; i<faces.cols(); ++i) {
      ofs << "f "
          << faces(0,i) << "/" << texture_map->find(faces(0,i))->second << " "
          << faces(1,i) << "/" << texture_map->find(faces(1,i))->second << " "
          << faces(2,i) << "/" << texture_map->find(faces(2,i))->second << endl;
    }
  } else {
    for (int i=0; i<faces.cols(); ++i) {
      ofs << "f " << faces(0,i) << " " << faces(1,i) << " " << faces(2,i) << endl;
    }
  }
  ofs.close();
  return 0;
}
