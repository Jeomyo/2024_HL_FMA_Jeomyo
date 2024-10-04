#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose
from sklearn.cluster import DBSCAN
from std_msgs.msg import Float32MultiArray

class SCANCluster:
    def __init__(self):
        # LiDAR 데이터를 수신
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        # 클러스터의 꼭짓점과 중앙 좌표를 퍼블리시
        self.cluster_pub = rospy.Publisher("/clusters", PoseArray, queue_size=1)
        # 클러스터의 거리 정보를 퍼블리시
        self.distance_pub = rospy.Publisher("/cluster_distances", Float32MultiArray, queue_size=1)

        self.pc_np = None
        self.dbscan = DBSCAN(eps=0.3, min_samples=3)  # DBSCAN 클러스터링 파라미터 설정

        # 50Hz 주기 설정
        rate = rospy.Rate(30)  # 50Hz 설정
        while not rospy.is_shutdown():
            # 콜백 함수가 데이터를 수신하고 처리할 수 있도록 spin 호출
            rate.sleep()

    
    def callback(self, msg):
        # PointCloud2 데이터를 numpy 배열로 변환
        self.pc_np = self.pointcloud2_to_xyz(msg)

        # 거리 값만 추출 (point_np의 세 번째 열을 가져옴)
        distances = self.pc_np[:, 2]

        # 클러스터 경계의 꼭짓점 4개와 중앙 좌표를 퍼블리시
        cluster_corners_and_center = self.cluster(self.pc_np[:, :2])  # x, y 좌표만 사용
        self.cluster_pub.publish(cluster_corners_and_center)
        
        # Float32MultiArray로 변환하여 거리 정보를 퍼블리시
        distances_msg = Float32MultiArray()
        distances_msg.data = distances
        self.distance_pub.publish(distances_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        # PointCloud2 메시지를 처리하여 x, y, 거리를 numpy 배열로 변환 (z축 제외)
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = (point[0]**2 + point[1]**2)**0.5  # 거리 계산
            # 조건에 맞는 포인트만 저장 (필터링)
            if point[0] > 0 and point[1] > -6 and point[1] < 6 and point[2] > -0.78 and point[2] < 3 and dist < 60:
                point_list.append((point[0], point[1], dist))  # (x, y, 거리)

        point_np = np.array(point_list, np.float32)  # 리스트를 numpy 배열로 변환
        return point_np      

    def cluster(self, xy):
        # x, y 좌표만 사용하여 클러스터링
        db = self.dbscan.fit_predict(xy)  # 클러스터 인덱스 생성
        n_cluster = np.max(db) + 1  # 클러스터 개수

        cluster_msg = PoseArray()  # PoseArray 초기화
        cluster_msg.header.frame_id = "/map"
        cluster_msg.header.stamp = rospy.Time.now()

        # 각 클러스터의 경계를 계산하여 네모의 4개의 꼭짓점 및 중앙 좌표 추가
        for cluster in range(n_cluster):
            cluster_points = xy[db == cluster, :]  # 현재 클러스터의 포인트들

            if len(cluster_points) > 0:
                # 클러스터의 최소/최대 값 계산 (경계 포인트)
                x_min, y_min = np.min(cluster_points, axis=0)
                x_max, y_max = np.max(cluster_points, axis=0)

                # 클러스터 중앙 좌표 계산
                center_x = np.mean(cluster_points[:, 0])  # x 좌표의 평균
                center_y = np.mean(cluster_points[:, 1])  # y 좌표의 평균

                # 중앙 좌표를 Pose로 변환하여 추가
                center_pose = Pose()
                center_pose.position.x = center_x
                center_pose.position.y = center_y
                center_pose.position.z = 0  # z축은 0으로 고정
                cluster_msg.poses.append(center_pose)

                # 네모의 꼭짓점 4개 계산
                corners = np.array([
                   [x_min, y_min],
                    [x_min, y_max],
                    [x_max, y_min],
                    [x_max, y_max]
                ])

                # 각 꼭짓점을 Pose로 변환하여 PoseArray에 추가 (z축은 0으로 고정)
                for corner in corners:
                    pose = Pose()
                    pose.position.x = corner[0]
                    pose.position.y = corner[1]
                    pose.position.z = 0  # z축을 0으로 설정
                    cluster_msg.poses.append(pose)

        return cluster_msg

if __name__ == '__main__':
    # ROS 노드 초기화 및 콜백 함수 대기
    rospy.init_node('velodyne_clustering', anonymous=True)
    scan_cluster = SCANCluster()
    rospy.spin()
