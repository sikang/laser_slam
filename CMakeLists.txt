cmake_minimum_required(VERSION 2.8.3)
project(laser_slam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include( ${CMAKE_CURRENT_SOURCE_DIR}/csm/GetCSM.cmake )
download_and_compile_csm()

include_directories(include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS})



catkin_simple()

cs_add_executable(laser_slam src/laser_slam_node.cpp
  src/scan_utils.cpp
  src/occ_grid.cpp
  src/height_estimation.cpp
  src/canonical_scan.cpp)
target_link_libraries(laser_slam gtsam ${csm_LIBRARIES} gsl)

cs_install()

cs_export()
