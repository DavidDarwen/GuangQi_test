#include <fstream>
#include <iostream>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ros/ros.h"
#include <can_msgs/gq_car.h>
#include <can_msgs/LuCe.h>
#include <can_msgs/msg_for_sim.h>

using namespace std;
using namespace can_msgs;
using namespace message_filters;

void Callback(const can_msgs::gq_car::ConstPtr& Car_messages,const can_msgs::LuCe::ConstPtr& LuCe_messages)
{
    can_msgs::msg_for_sim MsgToSim;
    //car_part:
    MsgToSim.CheLiangXinXi.car_speed = Car_messages -> car_speed;
    MsgToSim.CheLiangXinXi.car_acc = Car_messages -> car_acc;
    MsgToSim.CheLiangXinXi.motor_speed= Car_messages -> motor_speed;
    MsgToSim.CheLiangXinXi.motor_torque = Car_messages -> motor_torque;
    MsgToSim.CheLiangXinXi.battery_soc = Car_messages -> battery_soc;
    MsgToSim.CheLiangXinXi.battery_voltage = Car_messages -> battery_voltage;
    MsgToSim.CheLiangXinXi.battery_current = Car_messages -> battery_current;
    MsgToSim.CheLiangXinXi.gas_pedal = Car_messages -> gas_pedal;
    MsgToSim.CheLiangXinXi.break_pedal = Car_messages -> break_pedal;
    MsgToSim.CheLiangXinXi.gear_leval = Car_messages -> gear_leval;
    MsgToSim.CheLiangXinXi.ready_status = Car_messages -> ready_status;

    //Road_part:
    MsgToSim.LuCeXinXi.TL_DurYellow = LuCe_messages -> TL_DurYellow;
    MsgToSim.LuCeXinXi.TL_DurRed= LuCe_messages -> TL_DurRed;
    MsgToSim.LuCeXinXi.TL_DurGreen = LuCe_messages -> TL_DurGreen;
    MsgToSim.LuCeXinXi.TL_WaitTime = LuCe_messages -> TL_WaitTime;
    MsgToSim.LuCeXinXi.TL_Status = LuCe_messages -> TL_Status;
    MsgToSim.LuCeXinXi.TL_DistanceFromStart = LuCe_messages -> TL_DistanceFromStart;

    //Output_for_test:
    ROS_INFO("This is car's current status:");
    ROS_INFO("The car_speed is: %f",MsgToSim.CheLiangXinXi.car_speed);
    ROS_INFO("The car_acc is: %f",MsgToSim.CheLiangXinXi.car_acc);
    ROS_INFO("The motor_speed is: %d",MsgToSim.CheLiangXinXi.motor_speed);
    ROS_INFO("The motor_torque is: %d",MsgToSim.CheLiangXinXi.motor_torque);
    ROS_INFO("The battery_soc is: %f",MsgToSim.CheLiangXinXi.battery_soc);
    ROS_INFO("The battery_voltage is: %d",MsgToSim.CheLiangXinXi.battery_voltage);
    ROS_INFO("The battery_current is: %d", MsgToSim.CheLiangXinXi.battery_current);
    ROS_INFO("The gas_pedal is: %f",MsgToSim.CheLiangXinXi.gas_pedal);
    ROS_INFO("The break_pedal is: %f", MsgToSim.CheLiangXinXi.break_pedal);
    ROS_INFO("The gear_leval is: %f",MsgToSim.CheLiangXinXi.gear_leval);
    ROS_INFO("The ready_status is: %f",MsgToSim.CheLiangXinXi.ready_status);

    ROS_INFO("This is road's current status:");
    ROS_INFO("The yellow light duration time is: %d",MsgToSim.LuCeXinXi.TL_DurYellow);
    ROS_INFO("The red light duration time is: %d",MsgToSim.LuCeXinXi.TL_DurRed);
    ROS_INFO("The green light duration time is: %d",MsgToSim.LuCeXinXi.TL_DurGreen);
    ROS_INFO("The current light keep time is: %d",MsgToSim.LuCeXinXi.TL_WaitTime);
    ROS_INFO("The light status is: %d",MsgToSim.LuCeXinXi.TL_Status);
    ROS_INFO("The distence from the front light is: %f",MsgToSim.LuCeXinXi.TL_DistanceFromStart);

    //Msg2Sim_pub.publish(MsgToSim);
}


int main(int argc,char **argv)
{
    ros::init(argc, argv, "trans_CarRoad_msg_to_sim");

    ros::NodeHandle nh;

    ros::Publisher Msg2Sim_pub = nh.advertise<can_msgs::msg_for_sim>("message_for_sim",1);

    // ros::Subscriber totalfault_sub = nh.subscriber("faultmsg",1,Callback);
    // ros::Subscriber totalwarn_sub = nh.subscriber("warnmsg",1,Callback);
    /*
    message_filters如何使用？
        message_filters::Subscriber<std_msgs::UInt32> sub(nh, "my_topic", 1);
        sub.registerCallback(myCallback);
    等价于：
        ros::Subscriber sub = nh.subscribe("my_topic", 1, myCallback);
        sub -> 订阅Subscriber的名称
        "my_topic" -> 订阅话题的名称
        myCallback -> 回调函数的名称
    */

    string GQ_Car_Status = "/GQ_Car_Status";
    string LuCe_Status = "/LuCe_Status";

    nh.getParam("GQ_Car_Status",GQ_Car_Status);
    nh.getParam("LuCe_Status",LuCe_Status);


    message_filters::Subscriber<can_msgs::gq_car> *GQ_Car_Status_sub;
    message_filters::Subscriber<can_msgs::LuCe> *LuCe_Status_sub;

    typedef message_filters::sync_policies::ApproximateTime<can_msgs::gq_car,
                                                            can_msgs::LuCe>
        MySyncPolicy_;

    message_filters::Synchronizer<MySyncPolicy_> *sync_;
    GQ_Car_Status_sub = new message_filters::Subscriber<can_msgs::gq_car>(nh, GQ_Car_Status, 1);
    LuCe_Status_sub = new message_filters::Subscriber<can_msgs::LuCe>(nh, LuCe_Status, 1);

    sync_ = new message_filters::Synchronizer<MySyncPolicy_>(MySyncPolicy_(10), *GQ_Car_Status_sub,
                                                                                *LuCe_Status_sub);
    sync_->setMaxIntervalDuration(ros::Duration(1.0));

    sync_->registerCallback(boost::bind(&Callback, _1, _2));

    ros::spin();

    return 0;
}