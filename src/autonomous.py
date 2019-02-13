#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy
import sys
from mav_msgs.msg import RateThrust
from autonomous_control.msg import *

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

class autonomous():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        pub_vel = rospy.Publisher('output/rateThrust', RateThrust, queue_size=2)
        pub_event = rospy.Publisher('universal_teleop/events', Event, queue_size=2)
        pub_control = rospy.Publisher('universal_teleop/controls', Control, queue_size=2)
        #pub_vel = n.advertise<mav_msgs::RateThrust>("output/rateThrust", 1);
        #pub_event = n.advertise<teleop::Event>("universal_teleop/events", 5);
        #pub_control = n.advertise<teleop::Control>("universal_teleop/controls", 1);
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('hurrdeedurr')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        autonomousNode = autonomous()
    except rospy.ROSInterruptException: pass
