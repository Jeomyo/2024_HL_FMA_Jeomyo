#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, sqrt, atan2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry  # Odometry 추가
from morai_msgs.msg import EgoVehicleStatus
from std_msgs.msg import String
import numpy as np

class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        # object_pointcloud_data 데이터를 받을 토픽을 명시
        rospy.Subscriber("/object_pointcloud_data", PointCloud, self.object_callback)

        # Local Path와 Vehicle Status 데이터를 수신할 Subscriber 설정
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)  # Odom에서 위치 정보 수신
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.status_callback)  # 속도만 받음
        rospy.Subscriber("/lane_position", String, self.lane_callback)

        self.lattice_path_pub = rospy.Publisher("/lattice_path", Path, queue_size=10)

        self.is_path = False
        self.is_status = False
        self.is_odom = False  # odom flag 추가
        self.is_obj = False
        self.local_path = None
        self.object_data = None
        self.lane_pos = None

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_status and self.is_odom:  # odom flag 확인
                if self.is_obj:
                    if self.check_collision(self.local_path, self.object_data):
                        lattice_path = self.latticePlanner(self.local_path)
                        lattice_path_index = self.collision_check(self.object_data, lattice_path)
                        # lattice 경로 메시지 Publish
                        print(lattice_path_index)
                        self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                    else:
                        self.lattice_path_pub.publish(self.local_path)
                else:
                    # 객체 데이터가 없으면 local_path 그대로 Publish
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def object_callback(self, msg):
        self.is_obj = True
        self.object_data = msg  # PointCloud 데이터를 저장

    def check_collision(self, ref_path, object_data):
        is_crash = False
        for point in object_data.points:
            for path in ref_path.poses:
                dis = sqrt((path.pose.position.x - point.x)**2 + (path.pose.position.y - point.y)**2)
                if dis < 2.4:  # 충돌 판단 거리 설정
                    is_crash = True
                    break
        return is_crash

    def collision_check(self, object_data, out_path):
        selected_lane = -1
        lane_weight = [6, 5, 4 ,3, 2, 1, 0, 1, 2, 3, 4 ,5, 6]  # 경로별 가중치

        for point in object_data.points:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
                    dis = sqrt(pow(point.x - path_pos.pose.position.x, 2) + pow(point.y - path_pos.pose.position.y, 2))
                    if dis < 2:   
                        lane_weight[path_num] += 100

        if self.lane_pos == "Right":
            # 왼쪽에 노란 차선이 있는 경우 왼쪽 경로에 높은 가중치 부여
            lane_weight[0] += 1000  # 가장 왼쪽 경로의 가중치 증가
            lane_weight[1] += 1000
            lane_weight[2] += 1000
            lane_weight[3] += 1000
            lane_weight[4] += 1000
            lane_weight[5] += 1000


        elif self.lane_pos == "Left":
            # 오른쪽에 노란 차선이 있는 경우 오른쪽 경로에 높은 가중치 부여  
            lane_weight[7] += 1000
            lane_weight[8] += 1000
            lane_weight[9] += 1000
            lane_weight[10] += 1000
            lane_weight[11] += 1000
            lane_weight[12] += 1000

        for i in range(len(lane_weight)):
            print(f"{i}: {lane_weight[i]}")


        selected_lane = lane_weight.index(min(lane_weight))
        return selected_lane

    def path_callback(self, msg):
        self.is_path = True
        self.local_path = msg

    def lane_callback(self, msg):
        self.lane_pos = msg.data

    def status_callback(self, msg):  # 속도만 처리
        self.is_status = True
        self.vehicle_velocity = msg.velocity.x * 3.6  # 속도를 km/h로 변환

    def odom_callback(self, msg):  # 차량 위치만 처리
        self.is_odom = True
        self.vehicle_pos_x = msg.pose.pose.position.x
        self.vehicle_pos_y = msg.pose.pose.position.y

    def latticePlanner(self, ref_path):
        out_path = []
        vehicle_pose_x = self.vehicle_pos_x  # Odometry에서 받은 차량 위치
        vehicle_pose_y = self.vehicle_pos_y  # Odometry에서 받은 차량 위치
        vehicle_velocity = self.vehicle_velocity  # EgoVehicleStatus에서 받은 차량 속도

        look_distance = int(vehicle_velocity * 0.2 * 2)
        
        if look_distance < 20 : # 최소 20m 설정
            look_distance = 20

        if len(ref_path.poses) > look_distance :
            # 좌표 변환 행렬 생성
            global_ref_start_point = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance*2].pose.position.x, ref_path.poses[look_distance*2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            # 회전 변환
            trans_matrix = np.array([
                [cos(theta), -sin(theta), translation[0]], 
                [sin(theta), cos(theta), translation[1]], 
                [0, 0, 1]
            ])

            # 역 변환
            det_trans_matrix = np.array([
                [trans_matrix[0][0], trans_matrix[1][0], -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                [trans_matrix[0][1], trans_matrix[1][1], -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                [0, 0, 1]
            ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            lane_off_set = [-10 ,-8, -5,-4, -3, -2, 0, 2, 3, 4, 5, 8, 10]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            # Lattice 경로 생성
            for end_point in local_lattice_points:
                lattice_path = Path()
                lattice_path.header.frame_id = '/map'

                waypoints_x = []
                waypoints_y = []

                x_interval = 0.5  # Point 간격
                x_start = 0
                x_end = end_point[0]
                y_start = local_ego_vehicle_position[1][0]
                y_end = end_point[1]

                x_num = x_end / x_interval
                for i in range(x_start, int(x_num)):
                    waypoints_x.append(i * x_interval)

                d = y_start
                c = 0
                b = 3 * (y_end - y_start) / x_end**2
                a = -2 * (y_end - y_start) / x_end**3

                for x in waypoints_x:
                    result = a * x**3 + b * x**2 + c * x + d
                    waypoints_y.append(result)

                # Local -> Global 변환 후 lattice path에 추가
                for i in range(len(waypoints_y)):
                    local_result = np.array([[waypoints_x[i]], [waypoints_y[i]], [1]])
                    global_result = trans_matrix.dot(local_result)

                    read_pose = PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0.0
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1

                    lattice_path.poses.append(read_pose)

                out_path.append(lattice_path)

            # 경로 Publish
            for i in range(len(out_path)):
                globals()['lattice_pub_{}'.format(i + 1)] = rospy.Publisher('/lattice_path_{}'.format(i + 1), Path, queue_size=1)
                globals()['lattice_pub_{}'.format(i + 1)].publish(out_path[i])

        return out_path


if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass
