#include "ros/ros.h"
#include <can_msgs/gq_car.h>

using namespace std;


void CarMsgsCallback(const can_msgs::gq_car& msg)
{
    ROS_INFO("This is car's current status:");
    ROS_INFO("The car_speed is: %f",msg.car_speed);
    ROS_INFO("The car_acc is: %f",msg.car_acc);
    ROS_INFO("The motor_speed is: %d",msg.motor_speed);
    ROS_INFO("The motor_torque is: %d",msg.motor_torque);
    ROS_INFO("The battery_soc is: %f",msg.battery_soc);
    ROS_INFO("The battery_voltage is: %d",msg.battery_voltage);
    ROS_INFO("The battery_current is: %d",msg.battery_current);
    ROS_INFO("The gas_pedal is: %f",msg.gas_pedal);
    ROS_INFO("The break_pedal is: %f",msg.break_pedal);
    ROS_INFO("The gear_leval is: %f",msg.gear_leval);
    ROS_INFO("The ready_status is: %f",msg.ready_status);

}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "test_listener");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("GQ_Car_Status", 1000, CarMsgsCallback);

    ros::spin();

    return 0;
}