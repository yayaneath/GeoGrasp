cmake_minimum_required(VERSION 2.8.3)
project(geograsp)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES GeoGrasp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(GeoGrasp
  lib/${PROJECT_NAME}/GeoGrasp.cpp
)

add_executable(cloud_processor src/cloud_processor.cpp)

target_link_libraries(cloud_processor
  GeoGrasp
  ${catkin_LIBRARIES}
)

install(TARGETS GeoGrasp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)