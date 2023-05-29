cmake_minimum_required(VERSION 2.8.3)
project(lidar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  common_msgs
  OpenCV
  cv_bridge
  pcl_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

generate_dynamic_reconfigure_options(
  cfg/Lidar.cfg
  #...
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
# CATKIN_DEPENDS roscpp rospy
# DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lidar.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(
  ${PROJECT_NAME}_cone_detector src/lidar_main.cpp src/lidar_handle.cpp src/lidar_pipeline.cpp
)
set_target_properties(
  ${PROJECT_NAME}_cone_detector PROPERTIES OUTPUT_NAME cone_detector PREFIX ""
)
add_dependencies(
  ${PROJECT_NAME}_cone_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}_cone_detector 
  ${PROJECT_NAME}_passthrough_filter 
  ${PROJECT_NAME}_ransac 
  ${PROJECT_NAME}_euclidian_clustering 
  ${PROJECT_NAME}_cone_reconstruction 
  ${PROJECT_NAME}_cone_identification
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_passthrough_filter src/lidar_passthrough_filter.cpp)
add_dependencies(${PROJECT_NAME}_passthrough_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_passthrough_filter ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_ransac src/lidar_ransac.cpp)
add_dependencies(${PROJECT_NAME}_ransac ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_ransac ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_euclidian_clustering src/lidar_euclidian_clustering.cpp)
add_dependencies(${PROJECT_NAME}_euclidian_clustering ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_euclidian_clustering ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_cone_reconstruction src/lidar_cone_reconstruction.cpp)
add_dependencies(${PROJECT_NAME}_cone_reconstruction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_cone_reconstruction ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_cone_identification src/lidar_cone_identification.cpp)
add_dependencies(${PROJECT_NAME}_cone_identification ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_cone_identification ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_cone_detector ${PROJECT_NAME}_gencfg)

catkin_install_python(PROGRAMS 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
