if (NOT MSVC)
    include(FindPkgConfig)
    pkg_check_modules(PC_CCORE "ccore")
    if (NOT PC_CCORE_FOUND)
        pkg_check_modules(PC_CCORE "ccore")
    endif (NOT PC_CCORE_FOUND)
    if (PC_CCORE_FOUND)
        # add CFLAGS from pkg-config file, e.g. draft api.
        add_definitions(${PC_CCORE_CFLAGS} ${PC_CCORE_CFLAGS_OTHER})
        # some libraries install the headers is a subdirectory of the include dir
        # returned by pkg-config, so use a wildcard match to improve chances of finding
        # headers and SOs.
        set(PC_CCORE_INCLUDE_HINTS ${PC_CCORE_INCLUDE_DIRS} ${PC_CCORE_INCLUDE_DIRS}/*)
        set(PC_CCORE_LIBRARY_HINTS ${PC_CCORE_LIBRARY_DIRS} ${PC_CCORE_LIBRARY_DIRS}/*)
    endif(PC_CCORE_FOUND)
endif (NOT MSVC)

find_path (
    CCORE_INCLUDE_DIRS
    NAMES ccore.h
    HINTS ${PC_CCORE_INCLUDE_HINTS}
)

find_library (
    CCORE_LIBRARIES
    NAMES ccore
    HINTS ${PC_CCORE_LIBRARY_HINTS}
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
    CCORE
    REQUIRED_VARS CCORE_LIBRARIES CCORE_INCLUDE_DIRS
)
mark_as_advanced(
    CCORE_FOUND
    CCORE_LIBRARIES CCORE_INCLUDE_DIRS
)
