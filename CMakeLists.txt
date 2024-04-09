cmake_minimum_required(VERSION 3.1)
project(mylib VERSION 1.0 LANGUAGES CXX)

add_library(mylib SHARED
  src/mylib.cpp
  src/mylib.h
)

set_target_properties(mylib PROPERTIES
  VERSION ${PROJECT_VERSION}
  PUBLIC_HEADER src/mylib.h
)

install(TARGETS mylib
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${CMAKE_BINARY_DIR}/mylib.pc DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
