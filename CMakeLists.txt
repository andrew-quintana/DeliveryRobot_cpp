# Minimum version of CMake required to build this project
cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project's name, version, and language
project(DeliveryRobot VERSION 1.0 LANGUAGES CXX)

# Set the include directories
set(EXTERNAL_LIB_DIR "/usr/local/lib")
set(EXTERNAL_INCLUDE_DIR "/usr/local/include")

# Find the AprilTag library
#add_library(apriltag SHARED IMPORTED GLOBAL)
find_package(apriltag REQUIRED)
include_directories(${apriltag_INCLUDE_DIRS})

# Find the OpenCV libraries
find_package(OpenCV REQUIRED)
include_directories("/usr/local/include/opencv4")

# Find the Boost library
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Find the GTest library
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Find the Eigen library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
    ${EXTERNAL_INCLUDE_DIR}
)

# Add source files to create a shared library named 'delivery_interface'
add_executable(delivery_interface SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/DeliveryFSM.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/DeliveryFSM.h
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/AprilTagSensor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/AprilTagSensor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/BoostGraph.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/BoostGraph.h
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/CameraCalibration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/CameraCalibration.h
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/ComputationalGeometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/ComputationalGeometry.h
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/Mapper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/Mapper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/OnlineSLAM.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/OnlineSLAM.h
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/Utilities.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/Utilities.h

)

# Set properties for the 'delivery_interface' target
set_target_properties(delivery_interface PROPERTIES
  VERSION ${PROJECT_VERSION}
  PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/DeliveryFSM.h
)

configure_file(delivery_interface.pc.in delivery_interface.pc @ONLY)

# Link libraries to the shared library
target_link_libraries(delivery_interface PRIVATE
    #apriltag
    ${AprilTag_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    Eigen3::Eigen
)

#[[
# Install the 'delivery_interface' library
install(TARGETS delivery_interface
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install the 'delivery_interface.pc' file
install(FILES ${CMAKE_BINARY_DIR}/delivery_interface.pc 
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig
)
]]#