
if(NOT DEFINED BITS)
  if(${CMAKE_SIZEOF_VOID_P} MATCHES "8")
	set(BITS "x64")
  else(${CMAKE_SIZEOF_VOID_P} MATCHES "8")
	set(BITS "32")
  endif(${CMAKE_SIZEOF_VOID_P} MATCHES "8")
endif(NOT DEFINED BITS)

#refere to https://en.wikipedia.org/wiki/Microsoft_Visual_C%2B%2B
if(MSVC)
  if(${MSVC_VERSION} MATCHES "1500")
    set(COMPILER "msvc2008")
  endif(${MSVC_VERSION} MATCHES "1500")
  if(${MSVC_VERSION} MATCHES "1600")
	set(COMPILER "msvc2010")
  endif(${MSVC_VERSION} MATCHES "1600")
  if(${MSVC_VERSION} MATCHES "1700")
    set(COMPILER "msvc2012")
  endif(${MSVC_VERSION} MATCHES "1700")
  if(${MSVC_VERSION} MATCHES "1800")
	set(COMPILER "msvc2013")
  endif(${MSVC_VERSION} MATCHES "1800")
  if(${MSVC_VERSION} MATCHES "1900")
	set(COMPILER "msvc2015")
  endif(${MSVC_VERSION} MATCHES "1900")
  if(${MSVC_VERSION} GREATER 1900)
    set(COMPILER "msvc2017")
  endif(${MSVC_VERSION} GREATER 1900)
endif(MSVC)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm.*|ARM.*)")
  set(PLATFORM "arm")
else(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm.*|ARM.*)")
  set(PLATFORM "x86")
endif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm.*|ARM.*)")

set(CMAKE_DEBUG_POSTFIX _${PLATFORM}_${COMPILER}_${BITS}d)
set(CMAKE_RELEASE_POSTFIX _${PLATFORM}_${COMPILER}_${BITS})
