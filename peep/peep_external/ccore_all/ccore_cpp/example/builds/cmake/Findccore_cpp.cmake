find_path (
    CCORE_CPP_INCLUDE_DIRS
    NAMES ccore_libbson.h
)

find_library (
    CCORE_CPP_LIBRARIES
    NAMES ccore_cpp
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
    CCORE_CPP
    REQUIRED_VARS CCORE_CPP_LIBRARIES CCORE_CPP_INCLUDE_DIRS
)
mark_as_advanced(
    CCORE_CPP_FOUND
    CCORE_CPP_LIBRARIES CCORE_CPP_INCLUDE_DIRS
)
