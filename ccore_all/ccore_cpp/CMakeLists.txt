########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.8.8)
project(ccore_cpp)
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


if(CMAKE_SYSTEM MATCHES "Linux")
        set(THREADS_PREFER_PTHREAD_FLAG ON)
        find_package(Threads REQUIRED)
        set(THREAD_LIBRARIES Threads::Threads)
endif()

########################################################################
# version
########################################################################
file (STRINGS "../VERSION" CCORE_CPP_VERSION)
message(STATUS "Detected CCORE_CPP Version - ${CCORE_CPP_VERSION}")

########################################################################
# includes
########################################################################

file(GLOB ccore_cpp_headers "${SOURCE_DIR}/include/*.h")
include_directories("${SOURCE_DIR}/include")

source_group ("Header Files" FILES ${ccore_cpp_headers})
install(FILES ${ccore_cpp_headers} DESTINATION include)


########################################################################
# souces
########################################################################

file(GLOB ccore_cpp_sources "${SOURCE_DIR}/src/*.cpp")

option(CCORE_CPP_BUILD_SHARED "Whether or not to build the shared object" ON)
option(CCORE_CPP_BUILD_STATIC "Whether or not to build the static archive" ON)

if (NOT CCORE_CPP_BUILD_SHARED AND NOT CCORE_CPP_BUILD_STATIC)
  message(FATAL_ERROR "Neither static nor shared library build enabled")
endif()

# shared
if (CCORE_CPP_BUILD_SHARED)
    add_library(ccore_cpp SHARED ${ccore_cpp_sources})

  set_target_properties (ccore_cpp PROPERTIES
    PUBLIC_HEADER "${public_headers}"
    DEFINE_SYMBOL "CCORE_CPP_EXPORTS"
    SOVERSION "0"
    VERSION "${CCORE_CPP_VERSION}"
    COMPILE_DEFINITIONS "DLL_EXPORT"
    OUTPUT_NAME "ccore_cpp"
    PREFIX "lib"
  )

  target_link_libraries(ccore_cpp
    ${CCORE_LIBRARIES} ${CZMQ_LIBRARIES} ${LIBZMQ_LIBRARIES} ${LIBBSON_LIBRARIES} ${MORE_LIBRARIES}
  )

  install(TARGETS ccore_cpp
    LIBRARY DESTINATION "lib${LIB_SUFFIX}" # .so file
    ARCHIVE DESTINATION "lib${LIB_SUFFIX}" # .lib file
    RUNTIME DESTINATION bin                # .dll file
  )

  target_include_directories(ccore_cpp
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

# static
if (CCORE_CPP_BUILD_STATIC)
  add_library(ccore_cpp-static STATIC ${ccore_cpp_sources})

  set_target_properties(ccore_cpp-static PROPERTIES
    PUBLIC_HEADER "${public_headers}"
    COMPILE_DEFINITIONS "CCORE_CPP_STATIC"
    OUTPUT_NAME "ccore_cpp"
    PREFIX "lib"
  )

  target_link_libraries(ccore_cpp-static
    ${CCORE_LIBRARIES} ${CZMQ_LIBRARIES} ${LIBZMQ_LIBRARIES} ${LIBBSON_LIBRARIES} ${MORE_LIBRARIES}
  )

  install(TARGETS ccore_cpp-static
    LIBRARY DESTINATION "lib${LIB_SUFFIX}" # .so file
    ARCHIVE DESTINATION "lib${LIB_SUFFIX}" # .lib file
    RUNTIME DESTINATION bin                # .dll file
  )

  target_include_directories(ccore_cpp-static
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_compile_definitions(ccore_cpp-static
    PUBLIC CCORE_STATIC
  )

endif()


########################################################################
# executables
########################################################################
add_executable(
    ccore_cpp_test_basic    
    "${SOURCE_DIR}/src/test/basic.cpp"
)
if (TARGET ccore_cpp-static)
target_link_libraries(
    ccore_cpp_test_basic    
    ccore_cpp-static
    ${CCORE_LIBRARIES}
    ${CZMQ_LIBRARIES}
    ${LIBZMQ_LIBRARIES}
    ${LIBBSON_LIBRARIES}
    ${THREAD_LIBRARIES}
    ${OPTIONAL_LIBRARIES}
)
endif()
install(TARGETS ccore_cpp_test_basic
    RUNTIME DESTINATION bin
)

add_executable(
    ccore_cpp_test_basic_poll    
    "${SOURCE_DIR}/src/test/basic_poll.cpp"
)
if (TARGET ccore_cpp-static)
target_link_libraries(
    ccore_cpp_test_basic_poll
    ccore_cpp-static
    ${CCORE_LIBRARIES}
    ${CZMQ_LIBRARIES}
    ${LIBZMQ_LIBRARIES}
    ${LIBBSON_LIBRARIES}
    ${THREAD_LIBRARIES}
    ${OPTIONAL_LIBRARIES}
)
endif()
install(TARGETS ccore_cpp_test_basic
    RUNTIME DESTINATION bin
)

########################################################################
# tests
########################################################################
set(CLASSTEST_TIMEOUT 60 CACHE STRING "Timeout of the selftest of a class")
set(TOTAL_TIMEOUT 600 CACHE STRING "Timout of the total testsuite")

########################################################################
########################################################################
# tests
########################################################################
set(CLASSTEST_TIMEOUT 60 CACHE STRING "Timeout of the selftest of a class")
set(TOTAL_TIMEOUT 600 CACHE STRING "Timout of the total testsuite")

########################################################################
# summary
########################################################################
message ("")
message (STATUS "******************* Configuration Summary *******************")
message (STATUS "General:")
message (STATUS "  Version           :   ${VERSION}")
message (STATUS "  System            :   ${CMAKE_SYSTEM_NAME}")
message (STATUS "  C compiler        :   ${CMAKE_C_COMPILER}")
message (STATUS "  Debug C flags     :   ${CMAKE_C_FLAGS_DEBUG} ${CMAKE_C_FLAGS}")
message (STATUS "  Release C flags   :   ${CMAKE_C_FLAGS_RELEASE} ${CMAKE_C_FLAGS}")
message (STATUS "  Build type        :   ${CMAKE_BUILD_TYPE}")
message (STATUS "  Static build      :   ${CCORE_CPP_BUILD_STATIC}")
message (STATUS "  Shared build      :   ${CCORE_CPP_BUILD_SHARED}")
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
message (STATUS "  'make test'           run the project's selftest")
message (STATUS "  'make install'        install the project to ${CMAKE_INSTALL_PREFIX}")
message (STATUS "")
