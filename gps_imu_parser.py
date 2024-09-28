#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import tf
import os
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from pyproj import Proj
from math import pi


class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        
        # GPS 구독
        self.gps_sub = rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        
        # 초기화
        self.x, self.y = None, None
        self.is_imu = True
        self.is_gps = False

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id = '/base_link'

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            os.system('clear')
            if self.is_gps:
                self.convertLL2UTM()
                self.odom_pub.publish(self.odom_msg)
                rospy.loginfo("odom_msg is now being published at '/odom' topic!")
                rospy.loginfo('-----------------[ odom_msg ]---------------------')
                rospy.loginfo(self.odom_msg.pose)
            if not self.is_imu:
                rospy.logwarn("[1] can't subscribe '/imu' topic... please check your IMU sensor connection")
            if not self.is_gps:
                rospy.logwarn("[2] can't subscribe '/gps' topic... please check your GPS sensor connection")
            
            self.is_gps = False
            rate.sleep()

    def navsat_callback(self, gps_msg):
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.is_gps = True

    def convertLL2UTM(self):    
        xy_zone = self.proj_UTM(self.lon, self.lat)

        if self.lon == 0 and self.lat == 0:
            self.x = 0.0
            self.y = 0.0
        else:
            self.x = xy_zone[0]
            self.y = xy_zone[1]

            self.odom_msg.header.stamp = rospy.get_rostime()
            self.odom_msg.pose.pose.position.x = self.x
            self.odom_msg.pose.pose.position.y = self.y
            self.odom_msg.pose.pose.position.z = 0.

    def imu_callback(self, data):
        if data.orientation.w == 0:
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w

        self.is_imu = True


if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
