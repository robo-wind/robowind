if (NOT MSVC)
    include(FindPkgConfig)
    pkg_check_modules(PC_CCORE_C "ccore")
    if (NOT PC_CCORE_C_FOUND)
        pkg_check_modules(PC_CCORE_C "ccore")
    endif (NOT PC_CCORE_C_FOUND)
    if (PC_CCORE_C_FOUND)
        # add CFLAGS from pkg-config file, e.g. draft api.
        add_definitions(${PC_CCORE_C_CFLAGS} ${PC_CCORE_C_CFLAGS_OTHER})
        # some libraries install the headers is a subdirectory of the include dir
        # returned by pkg-config, so use a wildcard match to improve chances of finding
        # headers and SOs.
        set(PC_CCORE_C_INCLUDE_HINTS ${PC_CCORE_C_INCLUDE_DIRS} ${PC_CCORE_C_INCLUDE_DIRS}/*)
        set(PC_CCORE_C_LIBRARY_HINTS ${PC_CCORE_C_LIBRARY_DIRS} ${PC_CCORE_C_LIBRARY_DIRS}/*)
    endif(PC_CCORE_C_FOUND)
endif (NOT MSVC)

find_path (
    CCORE_C_INCLUDE_DIRS
    NAMES ccore_c99.h
    HINTS ${PC_CCORE_C_INCLUDE_HINTS}
)

find_library (
    CCORE_C_LIBRARIES
    NAMES ccore_c
    HINTS ${PC_CCORE_C_LIBRARY_HINTS}
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
    CCORE_C
    REQUIRED_VARS CCORE_C_LIBRARIES CCORE_C_INCLUDE_DIRS
)
mark_as_advanced(
    CCORE_C_FOUND
    CCORE_C_LIBRARIES CCORE_C_INCLUDE_DIRS
)
