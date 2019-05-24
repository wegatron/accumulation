find_path(ZSW_LIB_INCLUDE_DIR zsw_vtk_io.h
$ENV{HOME}/usr/zsw_lib/include
)

find_library(ZSW_LIBRARIES_DEBUG
NAMES io_commond
PATHS
$ENV{HOME}/usr/zsw_lib/lib)

find_library(ZSW_LIBRARIES_RELEASE
NAMES io_common
PATHS
$ENV{HOME}/usr/zsw_lib/lib
)

set(ZSW_LIBRARIES optimized;${ZSW_LIBRARIES_RELEASE};debug;${ZSW_LIBRARIES_DEBUG})