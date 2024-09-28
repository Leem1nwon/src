#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

    # !!!!!!!!!!!!!!file path 수정 필요!!!!!!!!!!!!
    # file_path는 자율주행 경로(Path) 파일이 들어있는 폴더의 디렉토리를 지정
    
    #!!!!!!!!!!!!!gobal_path 파일 이름 수정 필요!!!!!!!!!!!!!!!
    # global_path = p_r.read_txt("ego_path.txt")에서 "ego_path.txt" 부분은 실제 자율주행 경로 파일의 이름으로 바꾸


class pathReader:
    

    def __init__(self,file_path):
        # rospack=rospkg.RosPack()
        # self.file_path=rospack.get_path(morai_msgs)
        self.file_path = file_path

    def read_txt(self,file_name):
        full_file_name = "/home/lmw/catkin_ws/src/gps_path/ego_path.txt"
        openFile = open(full_file_name,'r')
        out_path = Path()
        out_path.header.frame_id = '/map'
        #파일 한줄 --> waypoint 한개
        line=openFile.readlines()
        for i in line:
            tmp = i.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.position.z = 0.0
            read_pose.pose.orientation.x = 0
            read_pose.pose.orientation.y = 0
            read_pose.pose.orientation.z = 0
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)

        openFile.close()
        print(out_path)
        return out_path
    
if __name__ == '__main__':
    try:
        #!!!!!!!!!!!!!!!!!!paht 수정 필요!!!!!!!!!!!!!!111
        p_r = pathReader("/home/lmw/catkin_ws/src/gps_path")  # 원하는 파일 경로를 직접 지정
        global_path = p_r.read_txt("/home/lmw/catkin_ws/src/gps_path/ego_path.txt")
        rospy.init_node("path_reader", anonymous=True)
        path_pub = rospy.Publisher('/global_path',Path,queue_size=1)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            path_pub.publish(global_path)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass