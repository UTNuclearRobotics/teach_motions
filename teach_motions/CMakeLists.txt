cmake_minimum_required(VERSION 2.8.3)
project(teach_motions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  jog_arm
  roscpp
  rospy
  std_msgs
  tf
  tf2_geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS jog_arm tf tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(compliant_replay src/teach_motions/compliant_replay.cpp src/teach_motions/get_ros_params.cpp)
add_dependencies(compliant_replay ${catkin_EXPORTED_TARGETS})
target_link_libraries(compliant_replay ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)