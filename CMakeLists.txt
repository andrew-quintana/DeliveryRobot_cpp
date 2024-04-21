# Minimum version of CMake required to build this project
cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project's name, version, and language
project(delivery_module VERSION 1.0 LANGUAGES CXX)

# Set the include directories
set(SHARED_LIB_DIR "/usr/lib/aarch64-linux-gnu/")
set(SHARED_INCLUDE_DIR "/usr/include")
set(CMAKE_PREFIX_PATH "/usr" ${CMAKE_PREFIX_PATH})
set(CMAKE_INSTALL_PREFIX "/home/jetbot/libraries" ${CMAKE_INSTALL_PREFIX})
set(pybind11_DIR "/home/jetbot/.local/lib/python3.6/site-packages/pybind11/share/cmake/pybind11/")


# Find the required packages
find_package(pybind11 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
add_library(apriltag SHARED IMPORTED GLOBAL)
set_target_properties(apriltag PROPERTIES
    IMPORTED_LOCATION "/home/jetbot/libraries/lib/libapriltag.so"
    IMPORTED_LOCATION "/home/jetbot/libraries/lib/libapriltag.so.3"
    INTERFACE_INCLUDE_DIRECTORIES "${SHARED_INCLUDE_DIR}")


# Add source files to create a pybind11 module named 'delivery_module'
pybind11_add_module(delivery_module
	cpp/src/DeliveryFSM.cpp
	cpp/src/AprilTagSensor.cpp
	cpp/src/Astar.cpp
	cpp/src/AprilTagSensor.cpp
	cpp/src/BoostGraph.cpp
	cpp/src/CameraCalibration.cpp
	cpp/src/ComputationalGeometry.cpp
	cpp/src/Mapper.cpp
	cpp/src/OnlineSLAM.cpp
	cpp/src/Utilities.cpp
)


# Include directories
target_include_directories(delivery_module PRIVATE
    ${Boost_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
)

# Link libraries
target_link_libraries(delivery_module PRIVATE
    apriltag
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    Eigen3::Eigen
)

