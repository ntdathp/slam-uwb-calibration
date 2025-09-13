cmake_minimum_required(VERSION 3.0.2)
project(slam-uwb-calibration)

## C++ standard
add_compile_options(-std=c++17)

## Find catkin and system dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  uwb_driver
  rosbag
  pcl_conversions
  pcl_ros
)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

## Locate Google glog
find_path(GLOG_INCLUDE_DIR
  NAMES glog/logging.h
  PATHS /usr/include /usr/local/include
)
find_library(GLOG_LIBRARY
  NAMES glog
  PATHS /usr/lib /usr/local/lib
)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs uwb_driver rosbag pcl_conversions pcl_ros
  DEPENDS Ceres Eigen3 glog
)

## Include directories
include_directories(
  include
  include/factor
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIR}
)

## --------------------------------------------------------------------------------
## Add the AP_ESTIMATOR node
## --------------------------------------------------------------------------------
add_executable(ANCHOR_ESTIMATOR_NODE src/anchor_estimator.cpp)
target_link_libraries(ANCHOR_ESTIMATOR_NODE
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(ANCHOR_ESTIMATOR_NODE
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
install(TARGETS ANCHOR_ESTIMATOR_NODE
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## --------------------------------------------------------------------------------
## Add the ANCHOR_LOCALIZER node
## --------------------------------------------------------------------------------
add_executable(ANCHOR_LOCALIZER_NODE src/anchor_localizer.cpp)
target_link_libraries(ANCHOR_LOCALIZER_NODE
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GLOG_LIBRARY}
  OpenMP::OpenMP_CXX
)
add_dependencies(ANCHOR_LOCALIZER_NODE
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
install(TARGETS ANCHOR_LOCALIZER_NODE
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


