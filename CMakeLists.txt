# Minimum version of CMake required to build this project
cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project's name, version, and language
project(DeliveryRobot VERSION 1.0 LANGUAGES CXX)

# Set the include directories
set(CMAKE_PREFIX_PATH "/usr/local")
set(CMAKE_INSTALL_PATH "${CMAKE_PREFIX_PATH}/lib")
set(CMAKE_INSTALL_INCLUDE_PATH "${CMAKE_PREFIX_PATH}include")


# Add external libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(Boost REQUIRED COMPONENTS graph)
find_package(GTest REQUIRED)
add_library(apriltag SHARED IMPORTED GLOBAL)

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

# Set the PATH and include directories for each target
set_target_properties(delivery_module PROPERTIES
    INSTALL_PATH "${CMAKE_INSTALL_PATH}"
    INCLUDE_DIRECTORIES "${CMAKE_INSTALL_INCLUDE_PATH}")
    

# Set properties for each library
set_target_properties(apriltag PROPERTIES
    IMPORTED_LOCATION "${CMAKE_INSTALL_PATH}/libapriltag.so"
    IMPORTED_LOCATION "${CMAKE_INSTALL_PATH}/libapriltag.so.3"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_INSTALL_INCLUDE_PATH}")


# Include directories
target_include_directories(delivery_module PRIVATE
    ${Boost_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/extern
)

# Link libraries
target_link_libraries(delivery_module PRIVATE
    apriltag
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    Eigen3::Eigen
    ${CMAKE_INSTALL_PATH}
)

# Clean cached CMake rules via cmake --build . --target clean_cache
add_custom_target(clean_cache
                  COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeCache.txt
                  COMMENT "Deleting CMakeCache.txt")

