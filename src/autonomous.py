#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy
import sys
import message_filters

from rospy import Time
from mav_msgs.msg import RateThrust
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from autonomous_control.msg import *

class autonomous():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.beginning_time = rospy.get_time()
        self.idle_thrust = float(9.81)

        self.path_sub = message_filters.Subscriber('/pathplanning/input/rateThrust', RateThrust)
        self.imu_sub = message_filters.Subscriber("/uav/sensors/imu", Imu)
        self.height_sub_SYNCED = message_filters.Subscriber("/uav/sensors/downward_laser_rangefinder", Range)
        self.height_sub_UNSYNCED = rospy.Subscriber("/uav/sensors/downward_laser_rangefinder", Range, self.checkHeightCallback)

        ts = message_filters.ApproximateTimeSynchronizer([self.path_sub, self.imu_sub, self.height_sub_SYNCED], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)



        self.pub_vel = rospy.Publisher('output/rateThrust', RateThrust, queue_size=2)


    def checkHeightCallback(self,range):
        if range.range >= -1.5: #and (rospy.get_time() - self.beginning_time < 10):
            #In this stage, the imu is not sending out messages yet, so this condition should only happen at the beginning of the track
            #The goal of this conditional is to get the imu to publish a topic, and trigger the message_filter callback below
            print("it is at height range finder")
            msg = RateThrust()
            msg.header.frame_id = "uav/imu"
            msg.header.stamp = Time.now()
            msg.thrust.z = self.idle_thrust + 1;
            msg.angular_rates.x = 0
            msg.angular_rates.y = 0
            msg.angular_rates.z = 0

            self.pub_vel.publish(msg)
        else:
            print("it got to here")

    def callback(self,rate_thrust,imu,range):
        #This is where the pathing is actually done
        print(rospy.get_time())
        msg = RateThrust()
        msg.header.frame_id = rate_thrust.header.frame_id
        msg.header.stamp = Time.now()
        msg.thrust.z = rate_thrust.thrust.z;
        msg.angular_rates.x = rate_thrust.angular_rates.x
        msg.angular_rates.y = rate_thrust.angular_rates.y
        msg.angular_rates.z = rate_thrust.angular_rates.z

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
