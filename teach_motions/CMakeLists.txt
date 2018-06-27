cmake_minimum_required(VERSION 2.8.3)
project(teach_motions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  jog_arm
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  tf2_geometry_msgs
)

add_action_files(DIRECTORY action FILES ReplayMotion.action)

add_service_files(
  FILES
  RequestMotion.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS jog_arm std_msgs tf tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(compliant_replay src/teach_motions/compliant_replay.cpp src/teach_motions/get_ros_params.cpp)
target_link_libraries(compliant_replay ${catkin_LIBRARIES})
add_dependencies(compliant_replay teach_motions_gencpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)