#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy
import sys
import message_filters

from rospy import Time
from mav_msgs.msg import RateThrust
from sensor_msgs.msg import Range
from std_msgs.msg import Float64

class autonomous():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.beginning_time = rospy.get_time()
        self.idle_thrust = float(9.81)

        self.path_sub = message_filters.Subscriber('/pathplanning/input/rateThrust', RateThrust)
        self.ratethrust_z_sub = message_filters.Subscriber("/rateThrustZ/control_effort", Float64)
        self.ratethrust_roll_sub = message_filters.Subscriber("/rateThrustPitch/control_effort", Float64)
        self.ratethrust_pitch_sub = message_filters.Subscriber("/rateThrustRoll/control_effort", Float64)
        self.ratethrust_yaw_sub = message_filters.Subscriber("/rateThrustYaw/control_effort", Float64)
        self.ratethrust_height_sub = message_filters.Subscriber("/rateThrustYaw/control_effort", Float64)

        self.pub_vel = rospy.Publisher('output/rateThrust', RateThrust, queue_size=2)

        ts = message_filters.ApproximateTimeSynchronizer([self.ratethrust_height_sub, self.path_sub, self.ratethrust_z_sub, self.ratethrust_roll_sub, self.ratethrust_pitch_sub, self.ratethrust_yaw_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

        time_to_start = 0.5
        d = rospy.Duration(time_to_start, 0)
        now = rospy.get_rostime()
        end_time = d + now

        msg = RateThrust()
        msg.header.frame_id = "uav/imu"
        msg.header.stamp = Time.now()
        msg.thrust.z = self.idle_thrust+0.1;
        msg.angular_rates.x = 0
        msg.angular_rates.y = 0
        msg.angular_rates.z = 0

        while rospy.get_rostime() < end_time:
            print("ITS STARTING")
            self.pub_vel.publish(msg)

    def callback(self,range,rate_thrust,pid_z,pid_roll,pid_pitch,pid_yaw):
        print(range.range)
        if range.range >= -1.0:
            msg = RateThrust()
            msg.header.frame_id = "uav/imu"
            msg.header.stamp = Time.now()
            msg.thrust.z = self.idle_thrust + 1;
            msg.angular_rates.x = 0
            msg.angular_rates.y = 0
            msg.angular_rates.z = 0

            self.pub_vel.publish(msg)
        else:
            print("Its here")
            msg = RateThrust()
            msg.header.frame_id = "uav/imu"
            msg.header.stamp = Time.now()
            msg.thrust.z = rate_thrust.thrust.z - pid_z.data
            msg.angular_rates.x = rate_thrust.angular_rates.x - pid_roll.data
            msg.angular_rates.y = rate_thrust.angular_rates.y - pid_pitch.data
            msg.angular_rates.z = rate_thrust.angular_rates.z - pid_yaw.data

            self.pub_vel.publish(msg)

if __name__ == '__main__':
    rospy.init_node('autonomous_control')

    try:
        autonomousNode = autonomous()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            #autonomousNode.ts.registerCallback()
            rate.sleep()
    except rospy.ROSInterruptException: pass
