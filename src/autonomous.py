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

        msg.thrust.z = self.idleThrust + 5;
        msg.angular_rates.x = 15
        msg.angular_rates.y = 15
        msg.angular_rates.z = 15

        self.pub_vel.publish(msg)
#     float pitch = last_joy_msg.axes[joy_axes["pitch"]];
#     float roll = -last_joy_msg.axes[joy_axes["roll"]];
#     float yaw = last_joy_msg.axes[joy_axes["yaw"]];
#     float vertical = last_joy_msg.axes[joy_axes["vertical"]];
#
#     /* check deadzones */
# //    if (std::abs(pitch) < joy_deadzones["pitch"]) pitch = 0;
# //    if (std::abs(yaw) < joy_deadzones["yaw"]) yaw = 0;
# //    if (std::abs(roll) < joy_deadzones["roll"]) roll = 0;
# //    if (std::abs(vertical) < joy_deadzones["vertical"]) vertical = 0;
#
#     msg.angular_rates.y = std::pow(pitch,3.0f) * axis_scales["pitch"];
#     msg.angular_rates.x = std::pow(roll, 3.0f) * axis_scales["roll"];
#     msg.angular_rates.z = std::pow(yaw,3.0f) * axis_scales["yaw"];
#     msg.thrust.z = std::pow(vertical, 3.0f) * axis_scales["vertical"] + idleThrust;
#
#   }
#   else if (key_override_enabled) {
#     msg.angular_rates.y = key_axes_state["pitch"] * axis_scales["pitch"];
#     msg.angular_rates.x = key_axes_state["roll"] * axis_scales["roll"];
#     msg.angular_rates.z = key_axes_state["yaw"] * axis_scales["yaw"];
#     msg.thrust.z = key_axes_state["vertical"] * axis_scales["vertical"] + idleThrust;
#   }
#     // Publish message.
#     // Might be an empty message if there is no override enabled.
#     pub_vel.publish(msg);
#



if __name__ == '__main__':
    rospy.init_node('hurrdeedurr')
    try:
        autonomousNode = autonomous()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            autonomousNode.control()
            rate.sleep()
    except rospy.ROSInterruptException: pass
