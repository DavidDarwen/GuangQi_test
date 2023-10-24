#include <sstream>
#include "ros/ros.h"
#include <can_msgs/LuCe.h>

using namespace std;


void LuCeMsgsCallback(const can_msgs::LuCe& msg)
{
    ROS_INFO("This is road's current status:");
    ROS_INFO("The yellow light duration time is: %d",msg.TL_DurYellow);
    ROS_INFO("The red light duration time is: %d",msg.TL_DurRed);
    ROS_INFO("The green light duration time is: %d",msg.TL_DurGreen);
    ROS_INFO("The current light keep time is: %d",msg.TL_WaitTime);
    ROS_INFO("The light status is: %d",msg.TL_Status);
    ROS_INFO("The distence from the front light is: %f",msg.TL_DistanceFromStart);

}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "lu_test_listener");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("LuCe_Status", 1000, LuCeMsgsCallback);

    ros::spin();

    return 0;
}