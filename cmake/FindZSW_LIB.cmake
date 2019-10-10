if(${CMAKE_SIZEOF_VOID_P} MATCHES "8" AND ${MSVC_VERSION} MATCHES "1900")
    set(ZSW_LIB_DIR "$ENV{HOME}/usr/zsw_lib")
elseif(${CMAKE_SIZEOF_VOID_P} MATCHES "4" AND ${MSVC_VERSION} MATCHES "1800")
    set(ZSW_LIB_DIR "$ENV{HOME}/usr/win32/zsw_lib")
endif(${CMAKE_SIZEOF_VOID_P} MATCHES "8" AND ${MSVC_VERSION} MATCHES "1900")

find_path(ZSW_LIB_INCLUDE_DIR zsw_vtk_io.h
$ENV{HOME}/usr/zsw_lib/include
${ZSW_LIB_DIR}/include
)

find_library(ZSW_LIBRARIES_DEBUG
NAMES io_commond
PATHS
$ENV{HOME}/usr/zsw_lib/lib
${ZSW_LIB_DIR}/lib)

find_library(ZSW_LIBRARIES_RELEASE
NAMES io_common
PATHS
$ENV{HOME}/usr/zsw_lib/lib
${ZSW_LIB_DIR}/lib
)

set(ZSW_LIBRARIES optimized;${ZSW_LIBRARIES_RELEASE};debug;${ZSW_LIBRARIES_DEBUG})