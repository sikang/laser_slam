cmake_minimum_required(VERSION 2.8.3)
project(laser_slam)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

catkin_simple()

cs_add_executable(laser_slam src/laser_slam_node.cpp
  src/scan_utils.cpp
  src/occ_grid.cpp
  src/height_estimation.cpp
  src/canonical_scan.cpp)
target_link_libraries(laser_slam gtsam)

cs_install()

cs_export()
