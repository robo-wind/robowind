cmake_minimum_required(VERSION 2.8.12)
project(ccore)
enable_language(CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# CMake version older that 3.1 don't understand above commands
if(CMAKE_VERSION VERSION_LESS "3.1") 
    add_compile_options(-std=c++11)
endif()
enable_testing()

set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/builds/cmake")
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# Select flags
SET(CMAKE_C_FLAGS_RELEASE "-O3")

# Will be used to add flags to pkg-config useful when apps want to statically link
set(pkg_config_libs_private "")

########################################################################
# options
########################################################################
if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE Debug)
endif ()

#The MSVC C compiler is too out of date,
#so the sources have to be compiled as c++
if (MSVC)
    enable_language(CXX)
    file(GLOB sources "${SOURCE_DIR}/src/*.cc")
    set_source_files_properties(
        ${sources}
        PROPERTIES LANGUAGE CXX
    )
    set(MORE_LIBRARIES ws2_32 Rpcrt4 Iphlpapi)
endif()

list(APPEND CMAKE_MODULE_PATH "${SOURCE_DIR}")
set(OPTIONAL_LIBRARIES)

########################################################################
# LIBZMQ dependency
########################################################################
find_package(libzmq REQUIRED)
IF (LIBZMQ_FOUND)
    include_directories(${LIBZMQ_INCLUDE_DIRS})
    list(APPEND MORE_LIBRARIES ${LIBZMQ_LIBRARIES})
    set(pkg_config_libs_private "${pkg_config_libs_private} -lzmq")
ELSE (LIBZMQ_FOUND)
    message( FATAL_ERROR "libzmq not found." )
ENDIF (LIBZMQ_FOUND)

########################################################################
# CZMQ dependency
########################################################################
find_package(czmq REQUIRED)
IF (CZMQ_FOUND)
    include_directories(${CZMQ_INCLUDE_DIRS})
    list(APPEND MORE_LIBRARIES ${CZMQ_LIBRARIES})
    set(pkg_config_libs_private "${pkg_config_libs_private} -lczmq")
ELSE (CZMQ_FOUND)
    message( FATAL_ERROR "czmq not found." )
ENDIF (CZMQ_FOUND)

########################################################################
# version
########################################################################
file (STRINGS "../VERSION" CCORE_VERSION)
message(STATUS "Detected CCORE Version - ${CCORE_VERSION}")

########################################################################
# includes
########################################################################

file(GLOB ccore_headers "${SOURCE_DIR}/include/*.h" "${SOURCE_DIR}/../ccore_bson/ccore_bson/include/*.h")
MESSAGE( STATUS "ccore_headers:         " ${ccore_headers} )

source_group ("Header Files" FILES ${ccore_headers})
install(FILES ${ccore_headers} DESTINATION include)

########################################################################
# library
########################################################################


file (GLOB ccore_sources "${SOURCE_DIR}/src/*.h" "${SOURCE_DIR}/src/*.cpp" "${SOURCE_DIR}/src/impl/*.h" "${SOURCE_DIR}/src/impl/*.cpp" "${SOURCE_DIR}/../ccore_bson/ccore_bson/src/*.cpp") 

include_directories("${SOURCE_DIR}/src" "${SOURCE_DIR}/include" "${SOURCE_DIR}/../ccore_bson/ccore_bson/include" "${SOURCE_DIR}/../ccore_bson/ccore_bson/src")
source_group("Source Files" FILES ${ccore_sources})

option(CCORE_BUILD_SHARED "Whether or not to build the shared object" ON)
option(CCORE_BUILD_STATIC "Whether or not to build the static archive" ON)

if (NOT CCORE_BUILD_SHARED AND NOT CCORE_BUILD_STATIC)
  message(FATAL_ERROR "Neither static nor shared library build enabled")
endif()

ADD_DEFINITIONS (-DZMQ_BUILD_DRAFT_API -DCZMQ_BUILD_DRAFT_API)

IF (NOT MSVC)
  # avoid building everything twice for shared + static
  # only on *nix, as Windows needs different preprocessor defines in static builds
  add_library (objects OBJECT ${ccore_sources})
  set_property(TARGET objects PROPERTY POSITION_INDEPENDENT_CODE ON)
ENDIF (NOT MSVC)

# shared
if (CCORE_BUILD_SHARED)
  IF (MSVC)
    add_library(ccore SHARED ${ccore_sources})
  ELSE (MSVC)
    add_library(ccore SHARED $<TARGET_OBJECTS:objects>)
  ENDIF (MSVC)

  set_target_properties (ccore PROPERTIES
    PUBLIC_HEADER "${public_headers}"
    DEFINE_SYMBOL "CCORE_EXPORTS"
    SOVERSION "0"
    VERSION "${CCORE_VERSION}"
    COMPILE_DEFINITIONS "DLL_EXPORT"
    OUTPUT_NAME "ccore"
    PREFIX "lib"
  )

  target_link_libraries(ccore
    ${ZEROMQ_LIBRARIES} ${MORE_LIBRARIES}
  )

  install(TARGETS ccore
    LIBRARY DESTINATION "lib${LIB_SUFFIX}" # .so file
    ARCHIVE DESTINATION "lib${LIB_SUFFIX}" # .lib file
    RUNTIME DESTINATION bin                # .dll file
  )

  target_include_directories(ccore
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

# static
if (CCORE_BUILD_STATIC)
  IF (MSVC)
    add_library(ccore-static STATIC ${ccore_sources})
  ELSE (MSVC)
    add_library(ccore-static STATIC $<TARGET_OBJECTS:objects>)
  ENDIF (MSVC)

  set_target_properties(ccore-static PROPERTIES
    PUBLIC_HEADER "${public_headers}"
    COMPILE_DEFINITIONS "CCORE_STATIC"
    OUTPUT_NAME "ccore"
    PREFIX "lib"
  )

  target_link_libraries(ccore-static
    ${ZEROMQ_LIBRARIES} ${MORE_LIBRARIES}
  )

  install(TARGETS ccore-static
    LIBRARY DESTINATION "lib${LIB_SUFFIX}" # .so file
    ARCHIVE DESTINATION "lib${LIB_SUFFIX}" # .lib file
    RUNTIME DESTINATION bin                # .dll file
  )

  target_include_directories(ccore-static
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_compile_definitions(ccore-static
    PUBLIC CCORE_STATIC
  )

endif()

########################################################################
# tests
########################################################################
#set(CLASSTEST_TIMEOUT 60 CACHE STRING "Timeout of the selftest of a class")
#set(TOTAL_TIMEOUT 600 CACHE STRING "Timout of the total testsuite")

#if(UNIX)
#    find_program(MEMORYCHECK_COMMAND valgrind)
#    set(MEMORYCHECK_COMMAND_OPTIONS "--leak-check=full --show-reachable=yes
#                                     --error-exitcode=1
#                                     --suppressions=src/.valgrind.supp")
#endif()
#
#set(TEST_CLASSES
#    ccore
#)

########################################################################
# cleanup
########################################################################
add_custom_target (distclean @echo Cleaning for source distribution)

set(cmake_generated ${CMAKE_BINARY_DIR}/CMakeCache.txt
                    ${CMAKE_BINARY_DIR}/cmake_install.cmake
                    ${CMAKE_BINARY_DIR}/Makefile
                    ${CMAKE_BINARY_DIR}/CMakeFiles
                    ${CMAKE_BINARY_DIR}/CTestTestfile.cmake
                    ${CMAKE_BINARY_DIR}/DartConfiguration.tcl
                    ${CMAKE_BINARY_DIR}/Testing
                    ${CMAKE_BINARY_DIR}/compile_commands.json
                    ${CMAKE_BINARY_DIR}/src/libccore.pc
                    ${CMAKE_BINARY_DIR}/src/libccore.so
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
message (STATUS "  Static build      :   ${CCORE_BUILD_STATIC}")
message (STATUS "  Shared build      :   ${CCORE_BUILD_SHARED}")
message (STATUS "")
message (STATUS "Dependencies:")
include(FeatureSummary)
feature_summary (WHAT ALL FATAL_ON_MISSING_REQUIRED_PACKAGES)
message (STATUS "")
message (STATUS "  Install prefix    :" "${CMAKE_INSTALL_PREFIX}")
message (STATUS "*************************************************************")
message (STATUS "Configuration complete! Now procced with:")
message (STATUS "  'make'                compile the project")
message (STATUS "  'make install'        install the project to ${CMAKE_INSTALL_PREFIX}")
message (STATUS "")

