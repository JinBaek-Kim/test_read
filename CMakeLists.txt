# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(opencv_read_evd)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "PCL library status:")
message(STATUS "    version: ${PCL_VERSION}")
message(STATUS "    libraries: ${PCL_LIBRARIES}")
message(STATUS "    include path: ${PCL_INCLUDE_DIRS}")

# [C/C++]>[General]>[Additional Include Directories]
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# [C/C++]>[Preprocessor]>[Preprocessor Definitions]
add_definitions( ${PCL_DEFINITIONS} )

# Declare the executable target built from your sources
add_executable(opencv_read_evd opencv_read_evd.cpp opencv_read_evd_func.cpp)

# Link your application with OpenCV, PCL libraries
target_link_libraries(opencv_read_evd ${OpenCV_LIBS} ${PCL_LIBRARIES})
