cmake_minimum_required(VERSION 2.8.3)
project(sync_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    tf2_ros
    nav_msgs
    roscpp
    rosconsole
    sensor_msgs
    #opencv
    image_transport
    dynamic_reconfigure
    urdf
    std_msgs
    rospy
    pcl_conversions
    pcl_ros
    tf
#    tf_conversion
    cv_bridge
    )

find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} "/usr/local/include")
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS} "/usr/local/lib")
ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CUDA_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sync_pkg
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(sync_node sync_node.cpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(sync_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${roslib_LIBRARIES}
 )

