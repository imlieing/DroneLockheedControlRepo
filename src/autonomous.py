#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy
import sys
from rospy import Time

from mav_msgs.msg import RateThrust
from autonomous_control.msg import *

class autonomous():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.idleThrust = float(9.81)
        self.pub_vel = rospy.Publisher('output/rateThrust', RateThrust, queue_size=2)
        self.pub_event = rospy.Publisher('universal_teleop/events', Event, queue_size=2)
        self.pub_control = rospy.Publisher('universal_teleop/controls', Control, queue_size=2)

    def control(self):
        msg = RateThrust()
        msg.header.frame_id = "uav/imu"
        msg.header.stamp = Time.now()

        msg.thrust.z = self.idleThrust + 1;
        msg.angular_rates.x = 0.05
        msg.angular_rates.y = 0.05
        msg.angular_rates.z = 0.05

        self.pub_vel.publish(msg)

if __name__ == '__main__':
    rospy.init_node('hurrdeedurr')
    try:
        autonomousNode = autonomous()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            autonomousNode.control()
            rate.sleep()
    except rospy.ROSInterruptException: pass
