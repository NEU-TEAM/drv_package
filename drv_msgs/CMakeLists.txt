cmake_minimum_required(VERSION 2.8.3)
project(drv_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

 add_message_files(
   FILES
   recognized_objects.msg
   recognized_target.msg
   target_info.msg
 )

 add_service_files(
   FILES
   recognize.srv
   user_select.srv
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})