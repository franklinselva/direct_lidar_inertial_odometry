###########################################################
#                                                         #
# Copyright (c)                                           #
#                                                         #
# The Verifiable & Control-Theoretic Robotics (VECTR) Lab #
# University of California, Los Angeles                   #
#                                                         #
# Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   #
# Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         #
#                                                         #
###########################################################

cmake_minimum_required(VERSION 3.12.4)
project(dlio VERSION 2.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(MPI COMPONENTS C)  # PCL's VTK dependency references MPI::MPI_C
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search registration surface octree)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>\nint main(){return 0;}")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)

# NanoFLANN
add_library(nanoflann STATIC src/nano_gicp/nanoflann.cc)
target_include_directories(nanoflann PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(nanoflann ${PCL_LIBRARIES})
set_property(TARGET nanoflann PROPERTY POSITION_INDEPENDENT_CODE ON)

# NanoGICP
add_library(nano_gicp STATIC
  src/nano_gicp/lsq_registration.cc
  src/nano_gicp/nano_gicp.cc)
target_include_directories(nano_gicp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(nano_gicp ${PCL_LIBRARIES} OpenMP::OpenMP_CXX nanoflann)
set_property(TARGET nano_gicp PROPERTY POSITION_INDEPENDENT_CODE ON)

# DLIO library
add_library(dlio SHARED
  src/dlio/config.cpp
  src/dlio/lio.cpp
  src/dlio/map.cpp
  src/dlio/io.cpp)
target_include_directories(dlio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(dlio PUBLIC
  ${PCL_LIBRARIES}
  yaml-cpp
  OpenMP::OpenMP_CXX
  Threads::Threads
  nano_gicp)
if(HAS_CPUID)
  target_compile_definitions(dlio PRIVATE HAS_CPUID)
endif()

add_library(dlio::dlio ALIAS dlio)

# Examples (own subdir so viz deps stay off the core lib)
option(DLIO_BUILD_EXAMPLES "Build example executables" ON)
if(DLIO_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# Smoke test
enable_testing()
add_executable(dlio_smoke_test tests/smoke_test.cpp)
target_link_libraries(dlio_smoke_test dlio)
add_test(NAME smoke COMMAND dlio_smoke_test)

# Install
install(TARGETS dlio nano_gicp nanoflann
  EXPORT dlioTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_DATADIR}/dlio)

install(EXPORT dlioTargets
  FILE dlioTargets.cmake
  NAMESPACE dlio::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dlio)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/dlioConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/dlioConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dlio)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/dlioConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/dlioConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/dlioConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dlio)
