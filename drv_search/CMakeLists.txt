cmake_minimum_required(VERSION 2.8.3)
project(drv_search)

aux_source_directory(./src SRC_LIST)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  drv_msgs
  geometry_msgs
  image_transport
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

catkin_package()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(drv_search ${SRC_LIST})

target_link_libraries(drv_search ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
