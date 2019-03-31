#!/usr/bin/env python

# Import required Python code.

import roslib
import rospy
import sys
import message_filters

from mav_msgs.msg import RateThrust
from sensor_msgs.msg import Imu

from std_msgs.msg import Float64

class pid():
    def __init__(self):
        self.path_sub = rospy.Subscriber('/pathplanning/input/rateThrust', RateThrust,self.publish_rate_thrust_as_float64)
        self.imu_sub = rospy.Subscriber("/uav/sensors/imu", Imu, self.publish_imu_as_float64)

        self.path_pub_z = rospy.Publisher("output/pathRateThrustZ", Float64, queue_size=2)
        self.path_pub_roll = rospy.Publisher("output/pathRateThrustRoll", Float64, queue_size=2)
        self.path_pub_pitch = rospy.Publisher("output/pathRateThrustPitch", Float64, queue_size=2)
        self.path_pub_yaw = rospy.Publisher("output/pathRateThrustYaw", Float64, queue_size=2)

        self.imu_pub_z = rospy.Publisher("output/imuRateThrustZ", Float64, queue_size=2)
        self.imu_pub_roll = rospy.Publisher("output/imuRateThrustRoll", Float64, queue_size=2)
        self.imu_pub_pitch = rospy.Publisher("output/imuRateThrustPitch", Float64, queue_size=2)
        self.imu_pub_yaw = rospy.Publisher("output/imuRateThrustYaw", Float64, queue_size=2)

    def publish_rate_thrust_as_float64(self, rate_thrust_msg):
        self.path_pub_z.publish(rate_thrust_msg.thrust.z)
        self.path_pub_roll.publish(rate_thrust_msg.angular_rates.x)
        self.path_pub_pitch.publish(rate_thrust_msg.angular_rates.y)
        self.path_pub_yaw.publish(rate_thrust_msg.angular_rates.z)

    def publish_imu_as_float64(self, imu_msg):
        self.imu_pub_z.publish(imu_msg.linear_acceleration.z)
        self.imu_pub_roll.publish(imu_msg.angular_velocity.x)
        self.imu_pub_pitch.publish(imu_msg.angular_velocity.y)
        self.imu_pub_yaw.publish(imu_msg.angular_velocity.z)




if __name__ == '__main__':
    rospy.init_node('pid_calculator')

    try:
        pidNode = pid()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            #autonomousNode.ts.registerCallback()
            rate.sleep()
    except rospy.ROSInterruptException: pass
