#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16

def send_data():
# 노드 초기화
    rospy.init_node('arduino_commander', anonymous=True)

    # 각 데이터를 보내는 topic 정의
    angle_pub = rospy.Publisher('/steering', Int16, queue_size=10)

    speed_pub = rospy.Publisher('/pwm', Int16, queue_size=10)

    # 1초 간격으로 데이터 전송
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        # 값 설정
        steering = 0

        pwm = 0

        # Publish 데이터를 보내기
        angle_pub.publish(steering)

        speed_pub.publish(pwm)

        rospy.loginfo(f"Sent: steering={steering}, pwm={pwm}")
        rate.sleep()

if __name__ == '__main__':
    try:
        send_data()
    except rospy.ROSInterruptException:
        pass