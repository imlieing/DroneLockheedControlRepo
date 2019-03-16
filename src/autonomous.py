#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy
import sys
import message_filters

from rospy import Time
from mav_msgs.msg import RateThrust
from sensor_msgs.msg import Range
from autonomous_control.msg import *
from std_msgs.msg import Float64

class autonomous():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.beginning_time = rospy.get_time()
        self.idle_thrust = float(9.81)
        #commented out because it will be moved to the pid node
        #self.path_sub = message_filters.Subscriber('/pathplanning/input/rateThrust', RateThrust)
        #self.imu_sub = message_filters.Subscriber("/uav/sensors/imu", Imu)
        #self.height_sub_SYNCED = message_filters.Subscriber("/uav/sensors/downward_laser_rangefinder", Range)


        self.height_sub_UNSYNCED = rospy.Subscriber("/uav/sensors/downward_laser_rangefinder", Range, self.checkHeightCallback)

        #these are the subscribers to the pid outputs
        self.ratethrust_z_sub = message_filters.Subscriber("/rateThrustZ/control_effort", Float64)
        self.ratethrust_roll_sub = message_filters.Subscriber("/rateThrustPitch/control_effort", Float64)
        self.ratethrust_pitch_sub = message_filters.Subscriber("/rateThrustRoll/control_effort", Float64)
        self.ratethrust_yaw_sub = message_filters.Subscriber("/rateThrustYaw/control_effort", Float64)

        self.pub_vel = rospy.Publisher('output/rateThrust', RateThrust, queue_size=2)

        ts = message_filters.ApproximateTimeSynchronizer([self.ratethrust_z_sub, self.ratethrust_roll_sub, self.ratethrust_pitch_sub, self.ratethrust_yaw_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)



    def checkHeightCallback(self,range):
        if range.range >= -1.0: #and (rospy.get_time() - self.beginning_time < 10):
            #In this stage, the imu is not sending out messages yet, so this condition should only happen at the beginning of the track
            #The goal of this conditional is to get the imu to publish a topic, and trigger the message_filter callback below
            #print("it is at height range finder")
            msg = RateThrust()
            msg.header.frame_id = "uav/imu"
            msg.header.stamp = Time.now()
            msg.thrust.z = self.idle_thrust + 1;
            msg.angular_rates.x = 0
            msg.angular_rates.y = 0
            msg.angular_rates.z = 0

            self.pub_vel.publish(msg)
        #else:
            #print("it got to here")

    def callback(self, pid_z,pid_roll,pid_pitch,pid_yaw):
        #print("FUCKKKKKKKKKKKKKKK")
        msg = RateThrust()
        #print(pid_z,pid_roll,pid_pitch,pid_yaw)
        msg.header.frame_id = "uav/imu"
        msg.header.stamp = Time.now()
        msg.thrust.z = pid_z.data
        msg.angular_rates.x = pid_roll.data
        msg.angular_rates.y = pid_pitch.data
        msg.angular_rates.z = pid_yaw.data

        self.pub_vel.publish(msg)

    #def subscribe_to_path_and_imu(self):
        #TO BE REPLACED WITH PATH TOPIC self.path_subscription = rospy.Subscriber("/bounding_box_camera/RGB", Image, process_path)

        #self.path_sub = rospy.Subscriber("TOPIC MADE FOR PATH", Path)
#path#

if __name__ == '__main__':
    rospy.init_node('autonomous_control')

    try:
        autonomousNode = autonomous()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            #autonomousNode.ts.registerCallback()
            rate.sleep()
    except rospy.ROSInterruptException: pass
