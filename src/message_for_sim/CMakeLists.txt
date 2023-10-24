cmake_minimum_required(VERSION 2.8.3)
project(message_for_sim)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  can_msgs
  message_filters
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES carmsgs_send_mivii
   CATKIN_DEPENDS std_msgs roscpp rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(Msg2Sim src/message_for_sim.cpp)
target_link_libraries(Msg2Sim ${catkin_LIBRARIES})
add_dependencies(Msg2Sim ${PROJECT_NAME}_generate_messages_cpp)
