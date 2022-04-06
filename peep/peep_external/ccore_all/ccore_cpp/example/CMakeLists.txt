cmake_minimum_required(VERSION 2.8.8)
project(ccore_cpp_sample)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()

set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/builds/cmake")
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# Select flags
SET(CMAKE_C_FLAGS_RELEASE "-O3")

# Will be used to add flags to pkg-config useful when apps want to statically link
set(pkg_config_libs_private "")

list(APPEND CMAKE_MODULE_PATH "${SOURCE_DIR}")
set(OPTIONAL_LIBRARIES)

set(CMAKE_FIND_LIBRARY_SUFFIXES ".a;.lib")

########################################################################
# LIBZMQ dependency
########################################################################
find_package(libzmq REQUIRED)
IF (LIBZMQ_FOUND)
    include_directories(${LIBZMQ_INCLUDE_DIRS})
ELSE (LIBZMQ_FOUND)
    message( FATAL_ERROR "libzmq not found." )
ENDIF (LIBZMQ_FOUND)

########################################################################
# CZMQ dependency
########################################################################
find_package(czmq REQUIRED)
IF (CZMQ_FOUND)
    include_directories(${CZMQ_INCLUDE_DIRS})
ELSE (CZMQ_FOUND)
    message( FATAL_ERROR "czmq not found." )
ENDIF (CZMQ_FOUND)

########################################################################
# LIBBSON dependency
########################################################################
find_package(libbson REQUIRED)
IF (LIBBSON_FOUND)
    include_directories(${LIBBSON_INCLUDE_DIRS})
ELSE (LIBBSON_FOUND)
    message( FATAL_ERROR "libbson not found." )
ENDIF (LIBBSON_FOUND)

########################################################################
# CCORE dependency
########################################################################
find_package(ccore REQUIRED)
IF (CCORE_FOUND)
    include_directories(${CCORE_INCLUDE_DIRS})
ELSE (CCORE_FOUND)
    message( FATAL_ERROR "ccore not found." )
ENDIF (CCORE_FOUND)


########################################################################
# CCORE_CPP dependency
########################################################################
find_package(ccore_cpp REQUIRED)
IF (CCORE_CPP_FOUND)
    include_directories(${CCORE_CPP_INCLUDE_DIRS})
ELSE (CCORE_CPP_FOUND)
    message( FATAL_ERROR "ccore_cpp not found." )
ENDIF (CCORE_CPP_FOUND)

if(CMAKE_SYSTEM MATCHES "Linux")
        set(THREADS_PREFER_PTHREAD_FLAG ON)
        find_package(Threads REQUIRED)
        set(THREAD_LIBRARIES Threads::Threads)
endif()

########################################################################
# version
########################################################################
# version
########################################################################
set(CCORE_LIBBSON_EXAMPLE_VERSION_MAJOR 0)
set(CCORE_LIBBSON_EXAMPLE_VERSION_MINOR 2)
set(CCORE_LIBBSON_EXAMPLE_VERSION_PATCH 0)
set(CCORE_LIBBSON_EXAMPLE_VERSION "${CCORE_LIBBSON_EXAMPLE_VERSION_MAJOR}.${CCORE_LIBBSON_EXAMPLE_VERSION_MINOR}.${CCORE_LIBBSON_EXAMPLE_VERSION_PATCH}")
message(STATUS "Detected CCORE_LIBBSON_EXAMPLE Version - ${CCORE_LIBBSON_EXAMPLE_VERSION}")

########################################################################
# executables
########################################################################
add_executable(
    ccore_libbson_ccore_example
    "${SOURCE_DIR}/src/ccore_example.cpp"
)

if (TARGET ccore_libbson_ccore_example)
target_link_libraries(
    ccore_libbson_ccore_example
    ${CCORE_CPP_LIBRARIES}
    ${CCORE_LIBRARIES}
    ${CZMQ_LIBRARIES}
    ${LIBZMQ_LIBRARIES}
    ${LIBBSON_LIBRARIES}
    ${THREAD_LIBRARIES}
    ${OPTIONAL_LIBRARIES}
)
endif()
add_executable(
    ccore_libbson_example
    "${SOURCE_DIR}/src/libbson_example.cpp"
)

if (TARGET ccore_libbson_example)
target_link_libraries(
    ccore_libbson_example
    ${CCORE_CPP_LIBRARIES}
    ${CCORE_LIBRARIES}
    ${CZMQ_LIBRARIES}
    ${LIBZMQ_LIBRARIES}
    ${LIBBSON_LIBRARIES}
    ${THREAD_LIBRARIES}
    ${OPTIONAL_LIBRARIES}
)
endif()
install(TARGETS ccore_libbson_ccore_example ccore_libbson_example 
    RUNTIME DESTINATION bin
)

########################################################################
# cleanup
########################################################################
add_custom_target (distclean @echo Cleaning for source distribution)

set(cmake_generated ${CMAKE_BINARY_DIR}/CMakeCache.txt
                    ${CMAKE_BINARY_DIR}/cmake_install.cmake
                    ${CMAKE_BINARY_DIR}/Makefile
                    ${CMAKE_BINARY_DIR}/CMakeFiles
                    ${CMAKE_BINARY_DIR}/CTestTestfile.cmake
)

add_custom_command(
    DEPENDS clean
    COMMENT "distribution clean"
    COMMAND rm
    ARGS    -rf CMakeTmp ${cmake_generated}
    TARGET  distclean
)

########################################################################
# summary
########################################################################
message ("")
message (STATUS "******************* Configuration Summary *******************")
message (STATUS "General:")
message (STATUS "  Version           :   ${VERSION}")
message (STATUS "  System            :   ${CMAKE_SYSTEM_NAME}")
message (STATUS "  C++ compiler      :   ${CMAKE_CXX_COMPILER}")
message (STATUS "  Debug C++ flags   :   ${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_CXX_FLAGS}")
message (STATUS "  Release C++ flags :   ${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS}")
message (STATUS "  Build type        :   ${CMAKE_BUILD_TYPE}")
message (STATUS "")
message (STATUS "Dependencies:")
include(FeatureSummary)
feature_summary (WHAT ALL FATAL_ON_MISSING_REQUIRED_PACKAGES)
message (STATUS "")
message (STATUS "Install:")
message (STATUS "  Install prefix    :" "${CMAKE_INSTALL_PREFIX}")
message (STATUS "")
message (STATUS "*************************************************************")
message (STATUS "Configuration complete! Now procced with:")
message (STATUS "  'make'                compile the project")
message (STATUS "  'make install'        install the project to ${CMAKE_INSTALL_PREFIX}")
message (STATUS "")
