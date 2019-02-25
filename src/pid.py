#!/usr/bin/env python

# Import required Python code.

import roslib
import rospy
import sys
import message_filters

from mav_msgs.msg import RateThrust
from sensor_msgs.msg import Imu
from autonomous_control.msg import *
from std_msgs.msg import Float64

class pid():
    def __init__(self):
        self.path_sub = message_filters.Subscriber('/pathplanning/input/rateThrust', RateThrust)
        self.imu_sub = message_filters.Subscriber("/uav/sensors/imu", Imu)

        self.pub_z = rospy.Publisher("output/rateThrustZ", Float64, queue_size=2)
        self.pub_roll = rospy.Publisher("output/rateThrustRoll", Float64, queue_size=2)
        self.pub_pitch = rospy.Publisher("output/rateThrustPitch", Float64, queue_size=2)
        self.pub_yaw = rospy.Publisher("output/rateThrustYaw", Float64, queue_size=2)


if __name__ == '__main__':
    rospy.init_node('pid_calculator')

    try:
        pidNode = pid()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            #autonomousNode.ts.registerCallback()
            rate.sleep()
    except rospy.ROSInterruptException: pass
