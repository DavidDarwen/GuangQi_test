cmake_minimum_required(VERSION 2.8.3)
project(can_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

##增加对 message_generation 的依赖
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)


#msg文件路径
add_message_files(
  DIRECTORY msg
  FILES
  Frame.msg
  gq_car.msg
)

##确保 CMake 知道在什么时候重新配置我们的 project,附加上所有你消息文件所依赖的那些含有 .msg 文件的 package(这个例子是依赖 std_msgs)
generate_messages(DEPENDENCIES std_msgs)

##你需要确保你设置了运行依赖
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)


####所有在 msg 路径下的.msg 文件都将转换为 ROS 所支持语言的源代码。生成的 C++头文件将会放置在 ~/catkin_ws/devel/include/beginner_tutorials/ 。
####lisp 文件会出现在 ~/catkin_ws/devel/share/common-lisp/ros/beginner_tutorials/msg/ 路径下.
