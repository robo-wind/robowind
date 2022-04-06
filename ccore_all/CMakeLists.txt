cmake_minimum_required(VERSION 2.8.12)

project(CommuniCore CXX)

option(BUILD_JUST_DEPS "Build just dependencies" OFF)
option(SYSTEM_INSTALL "Install in system directory" OFF)
option(FORCE_32BIT "Force 32 bit compilation" OFF)
option(FORCE_OSX_MEVERICKS "Set OSX Deployment target to 10.9 (Mevericks)" OFF)
option(LIBBSON_SUPPORT "Build CCore with libbson support (use only if you need something specific from libbson)" OFF)
option(C_SUPPORT "Build C SDK" OFF)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/builds/cmake"
  ${CMAKE_MODULE_PATH})

# Set up some standard defaults, these will be passed down into external the
# projects.
include(BuildType)
include(BuildLocation)
include(ExternalProject)

if (${SYSTEM_INSTALL})
    set(CommuniCore_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
else()
    set(CommuniCore_INSTALL_PREFIX "${CommuniCore_BINARY_DIR}/prefix")
endif()

if (${FORCE_32BIT})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
endif()

if (${FORCE_OSX_MEVERICKS})
  # https://stackoverflow.com/a/34208904
    set(EXTRA_OPTIONS "${EXTRA_OPTIONS} -DCMAKE_OSX_DEPLOYMENT_TARGET:STRING='10.9'")
endif()

set(CommuniCore_LIBRARY_SUFFIX ".a;.lib")
set(CommuniCore_DEFAULT_ARGS
  "-DCMAKE_PREFIX_PATH:PATH=${CommuniCore_INSTALL_PREFIX};${CMAKE_PREFIX_PATH}"
  "-DCMAKE_INSTALL_PREFIX:PATH=${CommuniCore_INSTALL_PREFIX}"
  "-DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}"
  "-DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}"
  "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
  "-DCMAKE_INSTALL_PREFIX:PATH=${CommuniCore_INSTALL_PREFIX}"
  "-DCMAKE_FIND_LIBRARY_SUFFIXES:STRING=${CommuniCore_LIBRARY_SUFFIX}"
  "-DENABLE_DRAFTS:BOOL=ON"
  ${EXTRA_OPTIONS}
)
   


message("suffix: ${CommuniCore_DEFAULT_ARGS}")

include(ExternalProjects-zmq)
include(ccore)


if (${LIBBSON_SUPPORT} OR ${C_SUPPORT})
    include(ExternalProjects-bson)
endif()

if (${LIBBSON_SUPPORT})
    include(ccore-libbson)
endif()

if (${C_SUPPORT})
    include(ccore-c)
endif()

if (NOT ${BUILD_JUST_DEPS})
    include(ccore-sample)
    if (${LIBBSON_SUPPORT})
        include(ccore-libbson-sample)
    endif()
    if (${C_SUPPORT})
        include(ccore-c-sample)
    endif()
    install( DIRECTORY ${CommuniCore_INSTALL_PREFIX}/lib 
             DESTINATION ".")
    install( DIRECTORY ${CommuniCore_INSTALL_PREFIX}/include
             DESTINATION "."
             PATTERN "z*" EXCLUDE
             PATTERN "czmq*" EXCLUDE
             PATTERN "slre*" EXCLUDE
             PATTERN "sha1*" EXCLUDE)
endif()

