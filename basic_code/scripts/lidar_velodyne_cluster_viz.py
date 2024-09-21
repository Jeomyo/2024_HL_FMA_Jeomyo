#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rospy
import numpy as np
import math
from sensor_msgs.msg import PointCloud
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Point32
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool

class Cluster_viz:

    def __init__(self):

        rospy.Subscriber("/clusters", PoseArray, self.callback)
        rospy.Subscriber("/dangerous_clusters", PoseArray, self.dan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/stop", Bool, self.stop_callback)
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.status_callback)

        self.object_pointcloud_pub = rospy.Publisher('object_pointcloud_data', PointCloud, queue_size=1)
        self.dynamic_pub = rospy.Publisher('dynamic_status', Bool, queue_size=1)


        self.is_odom = False
        self.cluster_status = False
        self.dangerous_status = False
        self.prev_dangerous_data = None
        self.cluster_data = None  # 초기화
        self.dangerous_data = None  # 초기화
        self.stop = False

        self.dynamic_status = Bool()
        self.dynamic_status.data = False

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():
            if self.is_odom and self.cluster_status:
                
                # (1) 좌표 변환 행렬 생성
                trans_matrix = self.trans_matrix(self.vehicle_yaw)

                
                obj_data_cluster = self.tf_global(trans_matrix, self.cluster_data)
                self.object_pointcloud_pub.publish(obj_data_cluster)

            rate.sleep()

    def trans_matrix(self, vehicle_yaw):
        # (1) 좌표 변환 행렬 생성
        trans_matrix = np.array([[math.cos(vehicle_yaw), -math.sin(vehicle_yaw), 0],
                                 [math.sin(vehicle_yaw), math.cos(vehicle_yaw), 0],
                                 [0, 0, 1]], dtype=np.float32)
        return trans_matrix
    
    def tf_global(self, trans_matrix, cluster_data):
        obj_data = PointCloud()
        obj_data.header.frame_id = 'map'
        obj_data.header.stamp = rospy.Time.now()  # 타임스탬프 추가
        obj_data.points = []  # 포인트 리스트 초기화

        vehicle_pos_x = self.vehicle_pos_x  # 루프 밖으로 이동
        vehicle_pos_y = self.vehicle_pos_y  # 루프 밖으로 이동

        for num, i in enumerate(cluster_data.poses):

            # (2) 전역 좌표로 변환
            local_result = [i.position.x, i.position.y, 1]
            temp = trans_matrix.dot(local_result)
            global_result = [temp[0] + vehicle_pos_x, temp[1] + vehicle_pos_y]

            # (3) 전역 좌표를 PointCloud에 추가
            tmp_point = Point32()
            tmp_point.x = global_result[0]
            tmp_point.y = global_result[1]
            tmp_point.z = 1.0  # 필요에 따라 수정 가능
            obj_data.points.append(tmp_point)

        return obj_data

    def check_if_dynamic(self, current_data, prev_data, threshold=0.2):
        for i, current_point in enumerate(current_data.points):
            prev_point = prev_data.points[i]

            dist = np.sqrt((current_point.x - prev_point.x) ** 2 +
                           (current_point.y - prev_point.y) ** 2)

            print(dist)               

            if dist > threshold:  # threshold 값을 기준으로 좌표 변화를 체크
                return True  # 동적인 물체

        return False  # 정적인 물체

    def callback(self, msg):    
        self.cluster_data = msg
        self.cluster_status = True

    def status_callback(self, msg):  # 속도만 처리
        self.is_status = True
        self.vehicle_velocity = msg.velocity.x * 3.6  # 속도를 km/h로 변환

    def dan_callback(self, msg):   
        self.dangerous_status = True 
        self.dangerous_data = msg 

        if self.vehicle_velocity < 0.01:
            trans_matrix = self.trans_matrix(self.vehicle_yaw)

            # (2) dangerous_data를 전역 좌표로 변환
            global_dangerous_data = self.tf_global(trans_matrix, self.dangerous_data)

            # (3) 이전 dangerous_data가 있으면 좌표 변화 체크
            if self.prev_dangerous_data is not None:
                self.dynamic_status.data = self.check_if_dynamic(global_dangerous_data, self.prev_dangerous_data)
                self.dynamic_pub.publish(self.dynamic_status)

            # 현재 dangerous_data를 prev_dangerous_data로 저장
            self.prev_dangerous_data = global_dangerous_data


    def stop_callback(self, msg):
        self.stop = msg.data

        if self.stop == False:
            self.prev_dangerous_data = None
            self.dangerous_status = False  # 위험 상태 해제


    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.vehicle_pos_x = msg.pose.pose.position.x
        self.vehicle_pos_y = msg.pose.pose.position.y

if __name__ == '__main__':

    rospy.init_node('velodyne_clustering', anonymous=True)

    Cluster_visualization = Cluster_viz()  # 오타 수정

    rospy.spin()
