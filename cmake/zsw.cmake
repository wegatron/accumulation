if(NOT DEFINED OS)
set(OS ${CMAKE_SYSTEM_NAME})
endif(NOT DEFINED OS)

if(NOT DEFINED BITS)
  if(${CMAKE_SIZEOF_VOID_P} MATCHES "8")
        set(BITS "64")
  else(${CMAKE_SIZEOF_VOID_P} MATCHES "8")
        set(BITS "32")
  endif(${CMAKE_SIZEOF_VOID_P} MATCHES "8")
endif(NOT DEFINED BITS)

macro(link_beta)
  message("-------$ENV{HOME}/usr_beta/${OS}/${BITS}/${COMPILER}/lib")
  link_directories(
        $ENV{HOME}/usr_beta/${OS}/lib
        $ENV{HOME}/usr_beta/${OS}/${BITS}/lib
        $ENV{HOME}/usr_beta/${OS}/${BITS}/${COMPILER}/lib
        )
endmacro(link_beta)


macro(include_beta)
  include_directories(
        $ENV{HOME}/usr_beta/include
        $ENV{HOME}/usr_beta/${OS}/include
        $ENV{HOME}/usr_beta/${OS}/${BITS}/include
        $ENV{HOME}/usr_beta/${OS}/${BITS}/${COMPILER}/include  # indeed, its better to be ${COMPILER}/${COMPILER_VERSION}
        )
endmacro(include_beta)
