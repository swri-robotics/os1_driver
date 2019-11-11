cmake_minimum_required(VERSION 2.8.3)
project(os1_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        image_transport
        message_generation
        nodelet
        pcl_ros
        pcl_conversions
        roscpp
        sensor_msgs
        std_msgs
        swri_nodelet
        tf2_ros
        tf2_geometry_msgs)

### Intel Threading Building Blocks ###
find_package(PkgConfig REQUIRED)
pkg_check_modules(TBB REQUIRED tbb)

### Boost ###
find_package(Boost REQUIRED)

### PCL ###
find_package(PCL REQUIRED COMPONENTS common)
add_definitions(${PCL_DEFINITIONS})



################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(FILES PacketMsg.msg)

## Generate services in the 'srv' folder
add_service_files(FILES OS1ConfigSrv.srv)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

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
        LIBRARIES os1_driver
        CATKIN_DEPENDS
          geometry_msgs
          image_transport
          message_runtime
          pcl_ros
          roscpp
          sensor_msgs
          std_msgs
          tf2_ros
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${TBB_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# main library
add_library(
        ${PROJECT_NAME}
        src/nodelets/os1_driver.cpp
        src/nodelets/os1_image.cpp
        src/nodelets/os1_cloud.cpp
        src/os1.cpp
        src/os1_ros.cpp
        src/os1_util.cpp
)
target_link_libraries(
        ${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${PCL_COMMON_LIBRARIES}
        ${TBB_LIBRARIES}
        ${catkin_LIBRARIES}
)
add_dependencies(
        ${PROJECT_NAME}
        # ${PROJECT_NAME}_gencpp
        ${catkin_EXPORTED_TARGETS}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
set_target_properties(
        ${PROJECT_NAME}
        PROPERTIES COMPILE_FLAGS "-std=c++14 -DPCL_NO_PRECOMPILE"
)

# node version of driver nodelet
swri_nodelet_add_node(os1_driver_node ${PROJECT_NAME} OS1Driver)
target_link_libraries(os1_driver_node ${PROJECT_NAME})
set_target_properties(os1_driver_node PROPERTIES COMPILE_FLAGS "-std=c++14 -DPCL_NO_PRECOMPILE")
add_dependencies(os1_driver_node ${PROJECT_NAME}_gencpp)

# node version of cloud nodelet
swri_nodelet_add_node(os1_packets_to_point_cloud_node ${PROJECT_NAME} OS1PacketsToPointCloud)
target_link_libraries(os1_packets_to_point_cloud_node ${PROJECT_NAME})
set_target_properties(os1_packets_to_point_cloud_node PROPERTIES COMPILE_FLAGS "-std=c++14 -DPCL_NO_PRECOMPILE")
add_dependencies(os1_packets_to_point_cloud_node ${PROJECT_NAME}_gencpp)

# node version of point cloud to image converter
swri_nodelet_add_node(os1_point_cloud_to_image_node ${PROJECT_NAME} OS1PointCloudToImage)
target_link_libraries(os1_point_cloud_to_image_node ${PROJECT_NAME})
set_target_properties(os1_point_cloud_to_image_node PROPERTIES COMPILE_FLAGS "-std=c++14 -DPCL_NO_PRECOMPILE")
add_dependencies(os1_point_cloud_to_image_node ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
    os1_driver_node
    os1_packets_to_point_cloud_node
    os1_point_cloud_to_image_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Plugin Definition File
install(
  FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Install launch files
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
