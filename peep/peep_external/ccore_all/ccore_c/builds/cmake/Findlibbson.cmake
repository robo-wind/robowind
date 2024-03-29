################################################################################
#  THIS FILE IS 100% GENERATED BY ZPROJECT; DO NOT EDIT EXCEPT EXPERIMENTALLY  #
#  Read the zproject/README.md for information about making permanent changes. #
################################################################################

if (NOT MSVC)
    include(FindPkgConfig)
    pkg_check_modules(PC_LIBBSON "libbson")
    if (NOT PC_LIBBSON_FOUND)
        pkg_check_modules(PC_LIBBSON "libbson")
    endif (NOT PC_LIBBSON_FOUND)
    if (PC_LIBBSON_FOUND)
        # add CFLAGS from pkg-config file, e.g. draft api.
        add_definitions(${PC_LIBBSON_CFLAGS} ${PC_LIBBSON_CFLAGS_OTHER})
        # some libraries install the headers is a subdirectory of the include dir
        # returned by pkg-config, so use a wildcard match to improve chances of finding
        # headers and SOs.
        set(PC_LIBBSON_INCLUDE_HINTS ${PC_LIBBSON_INCLUDE_DIRS} ${PC_LIBBSON_INCLUDE_DIRS}/*)
        set(PC_LIBBSON_LIBRARY_HINTS ${PC_LIBBSON_LIBRARY_DIRS} ${PC_LIBBSON_LIBRARY_DIRS}/*)
    endif(PC_LIBBSON_FOUND)
endif (NOT MSVC)

find_path (
    LIBBSON_INCLUDE_DIRS
    NAMES bson/bson.h
    HINTS ${PC_LIBBSON_INCLUDE_HINTS}
)

find_library (
    LIBBSON_LIBRARIES
    NAMES bson-static-1.0 
    HINTS ${PC_LIBBSON_LIBRARY_HINTS}
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
    LIBBSON
    REQUIRED_VARS LIBBSON_LIBRARIES LIBBSON_INCLUDE_DIRS
)
mark_as_advanced(
    LIBBSON_FOUND
    LIBBSON_LIBRARIES LIBBSON_INCLUDE_DIRS
)

################################################################################
#  THIS FILE IS 100% GENERATED BY ZPROJECT; DO NOT EDIT EXCEPT EXPERIMENTALLY  #
#  Read the zproject/README.md for information about making permanent changes. #
################################################################################
