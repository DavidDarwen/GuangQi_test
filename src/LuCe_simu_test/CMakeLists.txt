cmake_minimum_required(VERSION 2.8.3)
project(LuCe_simu_test)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  can_msgs
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

add_executable(LuCe_publisher src/LuCe_publisher.cpp)
target_link_libraries(LuCe_publisher ${catkin_LIBRARIES})
add_dependencies(LuCe_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_executable(Lu_test_subscriber src/subscriber.cpp)
target_link_libraries(Lu_test_subscriber ${catkin_LIBRARIES})
add_dependencies(Lu_test_subscriber ${PROJECT_NAME}_generate_messages_cpp)
