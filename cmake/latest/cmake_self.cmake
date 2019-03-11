# 因为机器上有不同版本的qt,设置当前使用的qt版本，只对vs有效.
set(ACTIVE_QT_DIR "$ENV{HOME}/usr/qt5.9/5.9.6/msvc2017_64")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${ACTIVE_QT_DIR}")

set(VTK_DIR "$ENV{HOME}/usr/VTK/lib/cmake/vtk-8.0")
set(FLANN_ROOT "$ENV{HOME}/usr/FLANN")
set(EIGEN_INCLUDE_DIRS "$ENV{HOME}/usr/eigen-3.3.5")
set(EIGEN3_INCLUDE_DIR "$ENV{HOME}/usr/eigen-3.3.5")
set(G2O_ROOT "$ENV{HOME}/usr/g2o")
set(BOOST_ROOT "$ENV{HOME}/usr/boost_1_68_0")
set(PCL_DIR "$ENV{HOME}/usr/pcl1.8.1")
# set(PCL_DIR "$ENV{HOME}/usr/pcl_1.8.1_new")
set(GTEST_ROOT "$ENV{HOME}/usr/gtest")
list(APPEND CMAKE_MODULE_PATH "$ENV{HOME}/usr/cmake_modules")
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/usr/pcl1.8.1/cmake")
# list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/usr/pcl1.8.1_new/cmake")
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/usr/opencv3.4.1/opencv/build")

# 对于无法安装cuda的机器,直接将cuda的库保存到一个目录并设置好该变量即可找到cuda
#set(CUDA_TOOLKIT_ROOT_DIR "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.0")
# set(ENV{LIBRARY_INSTALL_PATH} ${CMAKE_SOURCE_DIR}/../package)
#set(3rd_libs_dir ${CMAKE_SOURCE_DIR}/3rd)