cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(stereo)

# options
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")

# uncomment as required
# standard case
# set(PCL_DIR "$ENV{PCL_ROOT}/cmake" CACHE FILEPATH "The directory containing a CMake configuration file for PCL." )
# vcpkg installation
set(PCL_DIR "$ENV{HOME}/vcpkg/packages/pcl_x64-linux/share/pcl/" )

# Set C++ compilation flags
set(CMAKE_CXX_STANDARD 17)
set(GCC_COVERAGE_COMPILE_FLAGS \n
    "-DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules"
    #"-DOPENCV_EXTRA_MODULES_PATH=libs/opencv_contrib-4.x/modules"
    # add compile flags here if necessary
    )

# Add C++ compilation flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.12 REQUIRED)

# Additional Includes
# [C/C++]>[General]>[Additional Include Directories]
include_directories( ${PCL_INCLUDE_DIRS} )
# [C/C++]>[Preprocessor]>[Preprocessor Definitions]
add_definitions( ${PCL_DEFINITIONS} )
# [Linker]>[General]>[Additional Library Directories]
link_directories( ${PCL_LIBRARY_DIRS} )

# Define header and source files
set(HEADERS
    # add header files here
)

set(SOURCES
    src/main.cpp
)

add_executable(stereo ${HEADERS} ${SOURCES})
target_link_libraries(stereo Eigen3::Eigen)
target_link_libraries(stereo ${OpenCV_LIBS})
target_link_libraries(stereo ${PCL_LIBRARIES})
