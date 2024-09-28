#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import NavSatFix
import tf
from math import sqrt
from geometry_msgs.msg import PoseStamped
from path_reader_1 import pathReader
from std_msgs.msg import Int16
from math import degrees, atan2
import utm


# global_path와 turtle의 status_msg 이용해 현재 waypoint와 local_path 생성
def find_local_path(ref_path, current_position, index):
    out_path = Path()
    current_x = current_position[0]
    current_y = current_position[1]
    current_waypoint = index
    min_dls = float('inf')

    if current_waypoint + 10 > len(ref_path.poses):
        last_local_waypoint = len(ref_path.poses)
    else:
        last_local_waypoint = current_waypoint + 10

    # Find currently targeted waypoint by using previous index recursively
    for i in range(current_waypoint, last_local_waypoint):
        dx = current_x - ref_path.poses[i].pose.position.x
        dy = current_y - ref_path.poses[i].pose.position.y
        dls = sqrt(pow(dx, 2) + pow(dy, 2))
        if dls < min_dls:
            min_dls = dls
            current_waypoint = i   
        print(current_waypoint) 

    out_path.header.frame_id = 'map'
    for i in range(current_waypoint, last_local_waypoint):
        tmp_pose = PoseStamped()
        tmp_pose.pose.position.x = ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y = ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z = ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x = 0
        tmp_pose.pose.orientation.y = 0
        tmp_pose.pose.orientation.z = 0
        tmp_pose.pose.orientation.w = 1
        out_path.poses.append(tmp_pose)

    return out_path, current_waypoint

class ego_listener():
    def __init__(self):        
        rospy.Subscriber("/odom", Odometry, self.odom_callback)  # Odometry 데이터를 구독
        self.current_position = [0.0, 0.0]
        self.yaw = 0.0
    def odom_callback(self, data):
        # Odometry 메시지에서 현재 위치와 방향 정보 가져오기
        self.current_position = [data.pose.pose.position.x, data.pose.pose.position.y]

        # Orientation에서 Yaw 값 추출
        orientation_q = data.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # 보정된 Yaw 값을 계산
        corrected_yaw = yaw - tf.transformations.euler_from_quaternion([0, 0, self.initial_heading])[2]

        # TF 전송: 현재 위치와 방향(yaw)
        br = tf.TransformBroadcaster()
        br.sendTransform((self.current_position[0], self.current_position[1], 0),
                        tf.transformations.quaternion_from_euler(0, 0, corrected_yaw),
                        rospy.Time.now(),
                        "ego",
                        "map")

        

class PathSubscriber:
    def __init__(self):
        self.global_path = Path()
        self.is_path_received = False
        self.initial_heading = 0.0  # 초기 heading 값을 저장하는 변수
        rospy.Subscriber("/global_path", Path, self.global_path_callback)

    def calculate_initial_heading(self, start, end):
        """
        Calculate the heading from the first two waypoints
        """
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        heading = degrees(atan2(delta_y, delta_x))
        if heading < 0:
            heading += 360
        return heading

    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_path_received = True

        # 첫 두 웨이포인트로 초기 heading 계산
        first_point = [msg.poses[0].pose.position.x, msg.poses[0].pose.position.y]
        second_point = [msg.poses[1].pose.position.x, msg.poses[1].pose.position.y]
        self.initial_heading = self.calculate_initial_heading(first_point, second_point)
        rospy.loginfo(f"Initial heading from global path: {self.initial_heading} degrees")

if __name__ == '__main__':
    try: 
        rospy.init_node('local_path_finder', anonymous=True)
        local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        current_index_pub = rospy.Publisher('/current_waypoint', Int16, queue_size=1)
        el = ego_listener()

        path_sub = PathSubscriber()  # Create an instance of the PathSubscriber class
        current_waypoint = 0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if path_sub.is_path_received:  # Check if the global path is received
                # 지역경로생성
                local_path, current_waypoint = find_local_path(path_sub.global_path, el.current_position, current_waypoint)
                index_msg = Int16()
                index_msg.data = current_waypoint
                local_path_pub.publish(local_path)
                current_index_pub.publish(index_msg)

            else:
                rospy.logwarn("Global path has not been received yet. Waiting for the /global_path topic to publish data...")
                rate.sleep()
        

    except rospy.ROSInterruptException:
        pass