# GuangQi_test

代码使用流程：

1、启动ros：roscore

2、启动Miivii工控机CAN_0,1通道：nvidia@miivii-tegra:~/NSFC/sim1_0/src ros_socket_gqcan_driver$ sh gqcanpen_script.sh （链接到sh文件存放路径）

3_1、启动车辆信息ROS发布节点：rosrun ros_socket_gqcan_driver ros_socket_gqcan_driver_node

3_2、测试车辆信息ROS发布节点工作是否正常：rosrun ros_socket_gqcan_driver Che_test_subscriber

4_1、启动路侧信息ROS发布节点：rosrun LuCe_simu_test LuCe_publisher 

4_2、测试路侧信息ROS发布节点工作是否正常：rosrun LuCe_simu_test Lu_test_subscriber 

5、启动车路消息融合节点：rosrun message_for_sim Msg2Sim

6、执行主函数读取所有的仿真所需信息，进行节能算法运算

Made By DavidDarwen 2023.10.24
