# Minimum version of CMake required to build this project
cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project's name, version, and language
project(hardware_integration VERSION 1.0 LANGUAGES CXX)

# Set the include directories
set(CMAKE_PREFIX_PATH "/usr/local")
set(CMAKE_INSTALL_PATH "${CMAKE_PREFIX_PATH}/lib")
set(CMAKE_INSTALL_INCLUDE_PATH "${CMAKE_PREFIX_PATH}include")

# Add source files to create a pybind11 module named 'delivery_module'
add_executable(test    
    test.cpp
)

# Set the PATH and include directories for each target
set_target_properties(test PROPERTIES
    INSTALL_PATH "${CMAKE_INSTALL_PATH}"
    INCLUDE_DIRECTORIES "${CMAKE_INSTALL_INCLUDE_PATH}")
    
# Include directories
target_include_directories(test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/adafruitmotorhat
)
