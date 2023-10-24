cmake_minimum_required(VERSION 3.0.2)
project(ros_socket_gqcan_driver)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  message_generation
  can_msgs
)

# find_package(Boost REQUIRED COMPONENTS
#   system
# )
# add_package(yaml-cpp) by:zcj 20220310
# find_package(yaml-cpp  0.5.2  REQUIRED)

# add_message_files(
  # FILES
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/

)

add_executable(
  ros_socket_gqcan_driver_node

  src/ros_socket_gqcan_driver.cpp
  src/dbc_file/CAN_GQ.c
)

target_link_libraries(
  ros_socket_gqcan_driver_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.5.2
  /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.5

  # glog
)

add_dependencies(
  ros_socket_gqcan_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(Che_test_subscriber src/subscriber.cpp)
target_link_libraries(Che_test_subscriber ${catkin_LIBRARIES})
add_dependencies(Che_test_subscriber ${PROJECT_NAME}_generate_messages_cpp)
