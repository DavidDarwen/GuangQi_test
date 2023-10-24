#include <sstream>
#include "ros/ros.h"
#include <can_msgs/LuCe.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LuCe_talker");

    ros::NodeHandle nh;

    ros::Publisher LuCe_pub = nh.advertise<can_msgs::LuCe>("LuCe_Status",1000);

    ros::Rate loop_rate(10);

    int count = 0;

    while(ros::ok())
    {
        can_msgs::LuCe LuCe_message;

        LuCe_message.header.stamp = ros::Time::now();

        LuCe_message.TL_DurYellow = 5;
        LuCe_message.TL_DurRed = 25;
        LuCe_message.TL_DurGreen = 40;
        LuCe_message.TL_WaitTime = 20;

        LuCe_message.TL_Status = 1;

        LuCe_message.TL_DistanceFromStart = 120;

        LuCe_pub.publish(LuCe_message);

        ROS_INFO("LuCe Msgs Sending......");

        loop_rate.sleep();

        ++count;

    }

    return 0;

}