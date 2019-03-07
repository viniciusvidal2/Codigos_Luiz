cmake_minimum_required(VERSION 2.8.3)
project(zed_package)

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
             tf_conversions
             )
find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} "/usr/local/include")
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS} "/usr/local/lib")
ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

# ZED
find_package(ZED 2.1.2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zed_package
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
# include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(viewer viewer.cpp)
target_link_libraries(viewer ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${roslib_LIBRARIES})

add_executable(sincroniza_odom sincroniza_odom.cpp)
target_link_libraries(sincroniza_odom ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${roslib_LIBRARIES})

add_executable(zed_node zed_node.cpp)
target_link_libraries(zed_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${roslib_LIBRARIES} ${OpenCV_LIBS} ${ZED_LIBRARIES}
    ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})
