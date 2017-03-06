########################################################################
# Prevent in-tree builds
########################################################################
IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    MESSAGE(FATAL_ERROR "Prevented in-tree built. This is bad practice.")
ENDIF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

########################################################################
# Project setup
#######################################################################
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT (DYSPANRADIO)
ENABLE_TESTING()
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")
INCLUDE(CheckIncludeFileCXX)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" )
SET(CMAKE_CXX_FLAGS_RELEASE "-O2 -pipe -march=native -mtune=native -ffast-math -std=c++11" )
#-std=c++11 -O3 -mavx2
#-O3 -pipe -march=native -mtune=native -ffast-math -funsafe-math-optimizations -frename-registers -funroll-loops -ftree-vectorize -ftree-vectorizer-verbose=1 -Wall

add_definitions(-Wl,-unresolved-symbols=ignore-in-shared-libs)

########################################################################
# Setup UHD/Liquid/FFTW
########################################################################

FIND_PACKAGE(UHD REQUIRED)
FIND_PACKAGE(FFTW3F REQUIRED)
FIND_PACKAGE(LIQUIDDSP REQUIRED)
FIND_PACKAGE(PythonLibs REQUIRED)

########################################################################
# Setup Boost
########################################################################
MESSAGE(STATUS "")
MESSAGE(STATUS "Configuring Boost C++ Libraries...")
SET(BOOST_REQUIRED_COMPONENTS
    date_time
    program_options
    system
    thread
    filesystem
    unit_test_framework
)

IF(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")
    LIST(APPEND BOOST_LIBRARYDIR "/usr/lib64") #fedora 64-bit fix
ENDIF(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")

IF(MSVC)
    SET(BOOST_ALL_DYN_LINK "${BOOST_ALL_DYN_LINK}" CACHE BOOL "boost enable dynamic linking")
    IF(BOOST_ALL_DYN_LINK)
        ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK) #setup boost auto-linking in msvc
    ELSE(BOOST_ALL_DYN_LINK)
        SET(BOOST_REQUIRED_COMPONENTS) #empty components list for static link
    ENDIF(BOOST_ALL_DYN_LINK)
ENDIF(MSVC)

SET(Boost_ADDITIONAL_VERSIONS
    "1.46.0" "1.46" "1.47.0" "1.47" "1.48.0" "1.48" "1.48.0" "1.49" "1.50.0" "1.50"
    "1.51.0" "1.51" "1.52.0" "1.52" "1.53.0" "1.53" "1.54.0" "1.54" "1.55.0" "1.55"
    "1.56.0" "1.56"
)
FIND_PACKAGE(Boost 1.46 COMPONENTS ${BOOST_REQUIRED_COMPONENTS})

#find_library(PACKET_LIBRARY
#             NAMES packet
#             PATHS /home/connect/repo/dyspan_radio_2017/lib_x64/libpacket.so)
#link_directories(/home/connect/repo/dyspan_radio_2017/lib_x64)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS}
                    ${PYTHON_INCLUDE_DIR})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})# ${CMAKE_SOURCE_DIR}/clientlib)

MESSAGE(STATUS "Boost include directories: ${Boost_INCLUDE_DIRS}")
MESSAGE(STATUS "Boost library directories: ${Boost_LIBRARY_DIRS}")
MESSAGE(STATUS "Boost libraries: ${Boost_LIBRARIES}")

#LINK_DIRECTORIES(/home/connect/repo/dyspan_radio_2017/lib_x64)
#LINK_DIRECTORIES(/usr/local/lib)

# Adding source directory
#ADD_SUBDIRECTORY(clientlib)
ADD_SUBDIRECTORY(src)

ADD_SUBDIRECTORY(test)
