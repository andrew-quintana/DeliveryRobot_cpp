# Minimum version of CMake required to build this project
cmake_minimum_required(VERSION 3.1)

# Project's name, version, and language
project(mylib VERSION 1.0 LANGUAGES CXX)

# Add source files to create a shared library named 'mylib'
add_library(mylib SHARED
  src/mylib.cpp
  src/mylib.h
)

# Set properties for the 'mylib' target
set_target_properties(mylib PROPERTIES
  # Set the version of the library to the project's version
  VERSION ${PROJECT_VERSION}
  # Specify the public header file
  PUBLIC_HEADER src/mylib.h
)

# Install the 'mylib' library
install(TARGETS mylib
  # Specify the destination for the library file
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  # Specify the destination for the public header file
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install the 'mylib.pc' file
install(FILES ${CMAKE_BINARY_DIR}/mylib.pc 
  # Specify the destination for the 'mylib.pc' file
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig
)

# Set the path to the external library
set(EXTERNAL_LIB_DIR_1 "/usr/local/lib")
#set(EXTERNAL_LIB_DIR_2 "/path/to/external/library/2")
set(EXTERNAL_INCLUDE_DIR "/usr/local/include")

# Add the external libraries as imported targets
add_library(external_lib_1 SHARED IMPORTED)
set_target_properties(external_lib_1 PROPERTIES
    IMPORTED_LOCATION "${EXTERNAL_LIB_DIR_1}/libapriltag.so"
    INTERFACE_INCLUDE_DIRECTORIES "${EXTERNAL_INCLUDE_DIR}")

#add_library(external_lib_2 SHARED IMPORTED)
#set_target_properties(external_lib_2 PROPERTIES
#    IMPORTED_LOCATION "${EXTERNAL_LIB_DIR_2}/libexternal2.so"
#    INTERFACE_INCLUDE_DIRECTORIES "${EXTERNAL_INCLUDE_DIR}")

# Include the header files from the extern directory
include_directories(${EXTERNAL_INCLUDE_DIR})

# Link the external libraries to your target
target_link_libraries(mylib
	PUBLIC
		apriltag
		#external_lib_2
) 
