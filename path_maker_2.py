#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# 우리가 직접 path 만들어 볼 때 쓰는 코드... (자동으로 일정 거리마다 waypoint 찍어줌)


import rospy
import rospkg
from math import sqrt
# from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import NavSatFix
import utm




class pathMaker:


    def __init__(self,path_name):
        rospy.init_node('path_maker', anonymous=True)
        # /gps 토픽 구독

        
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.status_callback)
        # 초기화
        self.prev_x = 0
        self.prev_y = 0
        self.x_new = None
        self.y_new = None
        self.is_status=False
        # Save path to a specific directory

        # !!!!!!!! 폴더 디렉토리(저장할 파일명 제외, 저장할 위치가 되는 파일명) 변경하기!!!!!!!!!!
        save_directory = "/home/lmw/catkin_ws/src/gps_path"  # Replace with your desired path
        full_path = save_directory + '/' + path_name + ".txt"
        self.f= open(full_path, 'w')

        while not rospy.is_shutdown():
            if self.is_status==True:
                # GPS 위치 기록
                self.path_make()

        self.f.close()

    def path_make(self):

        x_pre = self.status_msg.longitude
        y_pre = self.status_msg.latitude
        utm_coord = utm.from_latlon(y_pre, x_pre)
        x=utm_coord[0]
        y=utm_coord[1]
        self.x_new = x
        self.y_new = y
        distance = sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        #이전 waypoint와 거리가 1.0 이상이면 기록
        if distance > 0.5:
            data='{0}\t{1}\n'.format(self.x_new,self.y_new)
            self.f.write(data)
            self.prev_x = x
            self.prev_y = y
            print(x,y)

    def status_callback(self,msg):
        self.is_status=True
        self.status_msg = msg
        #self.path_make()

if __name__ == '__main__':
    try:
        p_m = pathMaker("ego_path")
    except rospy.ROSInterruptException:
        pass
