import rospy
import time
from math import cos, sin, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int16
import numpy as np
from tf.transformations import euler_from_quaternion




class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("current_waypoint", Int16, self.index_callback)

        self.velocity_pub = rospy.Publisher('/velocity', Int16, queue_size=5)
        self.pwm_pub = rospy.Publisher('/pwm', Int16, queue_size=5)
        self.steering_pub = rospy.Publisher('/steering', Int16, queue_size=5)

        self.is_path = False
        self.is_odom = False
        self.is_index = False

        # self.initial_yaw_offset = None
        # self.heading = None # yaw_heading_cal.py d

        self.forward_point = Point()
        self.current_position = Point()
        self.current_waypoint = Int16()
        self.is_look_forward_point = False
        self.vehicle_length = 0.68
        self.lfd = 2
        self.steering = 0
        self.velocity = 0
        self.pwm = 0

        # steering average filter
        self.steering_buffer_size = 2
        self.steering_buffer = []

        self.parking_manager = ParkingManager()  # 주차 매니저 인스턴스 생성

        PARKING_PLACE_1 = -1
        PARKING_PLACE_2 = -1
        PARKING_PLACE_3 = -1


        rate = rospy.Rate(20)  # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_index:
                vehicle_position = self.current_position
                self.is_look_forward_point = False
                translation = [vehicle_position.x, vehicle_position.y]
                current_index = self.current_waypoint.data
                print("current waypoint : ", current_index)

                # 현재 차량 위치와 경로 변환 행렬 생성
                t = np.array([
                    [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                    [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                    [0, 0, 1]])
                det_t = np.array([
                    [t[0][0], t[1][0], -(t[0][0] * translation[0] + t[1][0] * translation[1])],
                    [t[0][1], t[1][1], -(t[0][1] * translation[0] + t[1][1] * translation[1])],
                    [0, 0, 1]])

                # 주차 구역에 최초로 진입했을 때
                # if not self.parking_manager.is_parking and (current_index == PARKING_PLACE_1 or current_index == PARKING_PLACE_2 or current_index == PARKING_PLACE_3):
                #     if current_index == PARKING_PLACE_1:
                #         self.parking_manager.start_parking(1)
                #     elif current_index == PARKING_PLACE_2:
                #         self.parking_manager.start_parking(2)
                    
                # # 주차 중일 때
                # if self.parking_manager.is_parking:
                #     self.steering, self.velocity = self.parking_manager.update_parking()

                if 1:#self.parking_manager.is_parking == False:   # 주차 끝내고 parking.reset_parking() 으로 주차모드 해제된 직후에도 purpusuit 위해 elif 말고 그냥if 사용
                    # 일반 주행 (Pure Pursuit)
                    for i, waypoint in enumerate(self.path.poses):
                        print(self.path.poses)
                        path_point = waypoint.pose.position
                        global_path_point = [path_point.x, path_point.y, 1]
                        local_path_point = det_t.dot(global_path_point)
                        # print(local_path_point[0])

                        # if local_path_point[0] != 0:
                        dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                        if  i>= current_index and dis >= self.lfd:
                            # self.forward_point = path_point
                            self.is_look_forward_point = True
                            print(f"Looking Index : {i}")
                            break
                        
                    theta = atan2(local_path_point[1], local_path_point[0])

                    print (f"is_look_forward_point : {self.is_look_forward_point}")

                    if self.is_look_forward_point:
                        # 속도 공식 : ks(상수) * base speedis_look
                        base_speed = 6
                        ks = 1 - 0.3 * (abs(self.steering) / 30) # 30도를 최대 steering각이라 했을 때 30퍼까지 감속되게
                        self.velocity = ks * base_speed

                        # self.lfd = 2.8 + (self.velocity - 7) / 5 # 속도에 따라서 lfd 거리 조절하기
                        print('Self.lfd : ', self.lfd)

                        raw_steering_angle = atan2((2 * self.vehicle_length * sin(theta)), self.lfd)*15 #* (4 / self.velocity)

                        if len(self.steering_buffer) < self.steering_buffer_size:
                            self.steering_buffer.append(raw_steering_angle)
                        else:
                            self.steering_buffer.pop(0)
                            self.steering_buffer.append(raw_steering_angle)

                        average_steering = sum(self.steering_buffer) / len(self.steering_buffer)
                        self.steering = self.apply_steering_threshold(average_steering)

                    else :
                        rospy.logwarn("No found forward point")
                        self.steering = 0.0
                        self.velocity = max(0.0, self.velocity - 0.1)

                
                self.pwm = 6*int(self.velocity)
                self.pwm = min(50, max(-50, self.pwm))
                self.steering = -int(min(30, max(-30, self.steering*4)))
                # print(self.steering)
                print('Steering : ',self.steering, end="")
                print('    /    PWM : ',self.pwm)
                self.steering_pub.publish(self.steering)
                self.velocity_pub.publish(self.velocity)
                self.pwm_pub.publish(self.pwm)

                self.is_path = False
                self.is_index = False
                self.is_odom = False
                self.is_look_forward_point = False

            elif not self.is_path:
                rospy.logwarn("Path received Fail")
            elif not self.is_odom:
                rospy.logwarn("Odom received Fail")
            elif not self.is_index:
                rospy.logwarn("Index received Fail")

            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def index_callback(self, msg):
        self.is_index = True
        self.current_waypoint = msg

    def apply_steering_threshold(self, steering_angle, threshold=0.05):
        if abs(steering_angle) < threshold:
            return 0
        return steering_angle
    

class ParkingManager:
    def __init__(self):
        self.parking_start_time = 0
        self.parking_timer = 0
        self.parking_mode = 0
        self.is_parking = False

    def start_parking(self, parking_mode):
        """주차 모드 시작"""
        self.parking_start_time = time.time()
        self.parking_mode = parking_mode
        self.is_parking = True

    def update_parking(self):
        """주차 구역 진입 후 경과 시간을 기반으로 주차 동작 결정"""
        self.parking_timer = time.time() - self.parking_start_time
        steering = 0
        velocity = 0

        # 첫 번째 주차 구역 (경사로)
        if self.parking_mode == 1:
            if 0 <= self.parking_timer < 3:
                velocity = 6
                steering = 0
            elif 3 <= self.parking_timer < 10:
                velocity = 0
                steering = 0
            elif  10<= self.parking_timer < 13:
                velocity = 255
                steering = 0
            else:
                self.reset_parking()

        # 두 번째 주차 구역 (T자 주차)
        elif self.parking_mode == 2:
            if 0 <= self.parking_timer < 3:
                velocity = -6
                steering = 0
            elif 3 <= self.parking_timer < 4:
                velocity = 0
                steering = -254
            elif  4<= self.parking_timer < 8:
                velocity = -254
                steering = 0
            elif  8<= self.parking_timer < 12:
                velocity = 0
                steering = 0
            elif  8<= self.parking_timer < 12:
                velocity = 0
                steering = 0
            elif  8<= self.parking_timer < 12:
                velocity = 0
                steering = 0
            elif  8<= self.parking_timer < 12:
                velocity = 0
                steering = 0
            else:
                self.reset_parking()
        
        # 세 번째 주차 구역
        elif self.parking_mode == 3:
            if 0 <= self.parking_timer < 3:
                velocity = -6
                steering = 0
            elif 3 <= self.parking_timer < 6:
                velocity = -6
                steering = 0
            else:
                self.reset_parking()

        return steering, velocity

    def reset_parking(self):
        """주차 모드 종료 및 초기화"""
        self.parking_start_time = 0
        self.parking_timer = 0
        self.parking_mode = 0
        self.is_parking = False


if __name__ == '__main__':
    try:
        pure_pursuit()
    except rospy.ROSInterruptException:
        pass
