INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_DBCONNECT dbconnect)

FIND_PATH(
    DBCONNECT_INCLUDE_DIRS
    NAMES dbconnect/api.h
    HINTS $ENV{DBCONNECT_DIR}/include
        ${PC_DBCONNECT_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    DBCONNECT_LIBRARIES
    NAMES gnuradio-dbconnect
    HINTS $ENV{DBCONNECT_DIR}/lib
        ${PC_DBCONNECT_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(DBCONNECT DEFAULT_MSG DBCONNECT_LIBRARIES DBCONNECT_INCLUDE_DIRS)
MARK_AS_ADVANCED(DBCONNECT_LIBRARIES DBCONNECT_INCLUDE_DIRS)

