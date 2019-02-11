cmake_minimum_required(VERSION 2.8.3)
project(villa_manipulation)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  control_msgs
  genmsg
  geometry_msgs
  hsr_kinematics
  message_generation
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  trajectory_msgs
  tmc_eigen_bridge
  tmc_manipulation_types_bridge
  tmc_manipulation_msgs
  tmc_planning_msgs
  tmc_utils
)

include_directories(
    ${catkin_INCLUDE_DIRS}
        include
)

add_service_files(
   FILES
   Opendoor.srv
   GetHandPositions.srv
   Grabbag.srv
   
)

add_action_files(
  DIRECTORY action
  FILES
   PerceptionPose.action
   PickUp.action
   PutDown.action
   ScanShelf.action
   Handover.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
   geometry_msgs
 )


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES libvilla_hsrb
  CATKIN_DEPENDS
  actionlib
  roscpp
  rospy
  trajectory_msgs
  control_msgs
  geometry_msgs
  sensor_msgs
  hsr_kinematics
  tmc_eigen_bridge
  tmc_manipulation_types_bridge
  tmc_utils
  actionlib_msgs

)

add_library(libvilla_hsrb src/libvilla_hsrb/manipulator.cpp)
target_link_libraries(libvilla_hsrb ${catkin_LIBRARIES})

add_executable(grasp_server src/grasp_server.cpp src/libvilla_hsrb/manipulator.cpp src/transform_utils.cpp)
add_dependencies(grasp_server ${villa_manipulation_EXPORTED_TARGETS})
target_link_libraries(grasp_server libvilla_hsrb ${catkin_LIBRARIES} )

add_executable(force_sensor src/force_sensor_test.cpp)
add_dependencies(force_sensor ${villa_manipulation_EXPORTED_TARGETS})
target_link_libraries(force_sensor libvilla_hsrb ${catkin_LIBRARIES})

add_executable(hsrb_wrapper_tester src/hsrb_wrapper_tester.cpp)
add_dependencies(hsrb_wrapper_tester ${villa_manipulation_EXPORTED_TARGETS})
target_link_libraries(hsrb_wrapper_tester libvilla_hsrb ${catkin_LIBRARIES})


install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

