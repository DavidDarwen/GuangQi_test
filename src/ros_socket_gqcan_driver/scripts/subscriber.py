#!/usr/bin/env python3
#coding=utf-8

import rospy
from can_msgs import gq_car

def gq_callback(msg):
    rospy.loginfo('The car_speed is: '+str(msg.car_speed))
    rospy.loginfo('The car_acc is: '+str(msg.car_acc))
    rospy.loginfo('The motor_speed is: '+str(msg.motor_speed))
    rospy.loginfo('The motor_torque is: '+str(msg.motor_torque))
    rospy.loginfo('The battery_soc is: '+str(msg.battery_soc))
    rospy.loginfo('The battery_voltage is: '+str(msg.battery_voltage))
    rospy.loginfo('The battery_current is: '+str(msg.battery_current))
    rospy.loginfo('The gas_pedal is: '+str(msg.gas_pedal))
    rospy.loginfo('The break_pedal is: '+str(msg.break_pedal))

if __name__ == '__main__':
    rospy.init_node("test_listener")

    sub = rospy.Subscriber("GQ_Car_Status",gq_car,gq_callback,queue_size = 10)

    rospy.spin()
