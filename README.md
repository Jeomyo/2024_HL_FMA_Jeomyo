# 2024 HL FMA - 정적 장애물 회피 기반 자율주행 시스템 (with MORAI)

## 📌 시스템 구조 (System Architecture)

아래는 센서부터 제어 명령까지 전체 처리 흐름을 보여주는 플로우차트입니다.
<p align="left">
  <img src="https://github.com/user-attachments/assets/a353038b-8bd2-40bd-abc0-087ef68d2ef9" alt="이미지 설명" width="600">
</p>

## 📌 패키지 상세 구성 및 주요 기능

# 1. GPS IMU 데이터 Publish

## 1) gpsimu_parser.py - GPS & IMU 데이터 파싱 및 Odometry 생성
|역할|GPS 및 IMU 데이터를 수신하고, UTM 좌표 변환 후 /odom으로 퍼블리시|
|------|------|
|구독 토픽| /gps (GPSMessage), /imu (Imu)|
|퍼블리시 토픽| /odom (Odometry)|
|주요 기능 요약| GPS 좌표를 UTM 좌표로 변환하여 위치 정보 제공, IMU에서 차량의 자세(Orientation) 정보 제공|

<details>
<summary> <b> 📌 gpsimu_parser 코드 분석 펼쳐보기 </b> </summary>
  
### 1. 노드 초기화 및 토픽 설정
  
```python
rospy.init_node('GPS_IMU_parser', anonymous=True)
self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
```
* /gps에서 GPS 데이터를 수신
* /imu에서 IMU 데이터를 수신 (자세 정보)
* /odom으로 최종 변환된 위치 및 자세 정보 퍼블리시
* 10Hz 주기로 동작

### 2. GPS 수신 및 저장 (navsat_callback)
```python
def navsat_callback(self, gps_msg):
    self.lat = gps_msg.latitude
    self.lon = gps_msg.longitude
    self.e_o = gps_msg.eastOffset
    self.n_o = gps_msg.northOffset
    self.is_gps = True
```
* GPSMessage 메시지를 받아서 위도(lat), 경도(lon), 동쪽 오프셋(eastOffset), 북쪽 오프셋(northOffset)을 저장
* 데이터가 들어왔음을 self.is_gps로 표시해 이후 처리 시 플래그로 활용

### 3. GPS 위도/경도를 UTM 좌표로 변환 (convertLL2UTM)
```python
xy_zone = self.proj_UTM(self.lon, self.lat)

if self.lon == 0 and self.lat == 0:
    self.x = 0.0
    self.y = 0.0
else:
    self.x = xy_zone[0] - self.e_o
    self.y = xy_zone[1] - self.n_o

self.odom_msg.header.stamp = rospy.get_rostime()
self.odom_msg.pose.pose.position.x = self.x
self.odom_msg.pose.pose.position.y = self.y
self.odom_msg.pose.pose.position.z = 0.
```
* UTM 좌표계 변환 (pyproj 라이브러리 활용)
* 변환된 UTM 좌표에서 GPS 오프셋(eastOffset, northOffset)을 보정
* 변환 결과를 /odom 메시지의 position에 저장

### 4. IMU 데이터 수신 및 orientation 저장 (imu_callback)
```python
if data.orientation.w == 0:
    self.odom_msg.pose.pose.orientation.x = 0.0
    self.odom_msg.pose.pose.orientation.y = 0.0
    self.odom_msg.pose.pose.orientation.z = 0.0
    self.odom_msg.pose.pose.orientation.w = 1.0
else:
    self.odom_msg.pose.pose.orientation.x = data.orientation.x
    self.odom_msg.pose.pose.orientation.y = data.orientation.y
    self.odom_msg.pose.pose.orientation.z = data.orientation.z
    self.odom_msg.pose.pose.orientation.w = data.orientation.w

self.is_imu = True
```
* /imu에서 수신한 orientation를 odom 메시지에 저장
* IMU 데이터가 들어온 상태를 self.is_imu 플래그로 표시

### 5. 메인 루프에서 GPS와 IMU 데이터 결합 후 Odometry 퍼블리시
```python
 if self.is_imu == True and self.is_gps == True:
    self.convertLL2UTM()
    self.odom_pub.publish(self.odom_msg)
```

* GPS와 IMU 데이터가 모두 수신되면, 좌표 변환 및 자세 결합 후 /odom 퍼블리시
* GPS, IMU 데이터 누락 시 경고 메시지 출력

### 📝 요약
|기능|설명|
|---|---|
|GPS 수신|/gps에서 위도, 경도, 오프셋 수신|
|IMU 수신|/imu에서 자세 데이터 수신|
|UTM 변환|위경도 → UTM 변환 및 오프셋 보정|
|Odometry 생성|위치+자세 결합 후 /odom 퍼블리시|
|주기적 동작|10Hz 주기로 데이터 수신 여부 체크 및 퍼블리시|
</details>




# 2. 글로벌 & 로컬 경로 계획

### 경로 계획이란?
* 목표 지점까지 최적의 경로로 도달하게 하는 기술
* 경로 계획은 크게 전역 경로 계획과 지역 경로 계획으로 나눌 수 있음.

| 구분 | 설명 |
|---|---|
| 전역 경로 계획 | 지도에 기반하여 경로를 자율적으로 생성 |
| 지역 경로 계획 | 전역 경로에서 얻은 구간을 주행하기 위하여 차량이 실제로 주행해야 할 경로 |


## 1) global_path_pub.py - 전역 경로 퍼블리시
|역할|저장된 경로 파일(mando_path.txt)을 읽어 /global_path로 퍼블리시|
|------|------|
|구독 토픽| X|
|퍼블리시 토픽| /global_path (Path)|
|주요 기능 요약|전역 경로 제공|
<details> <summary><b> 📌 global_path 코드 분석 펼쳐보기 </b></summary>

### 1. 전역 경로 파일 불러오기 
  
```python
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('beginner_tutorials')
full_path = pkg_path + '/path/mando_path.txt'

with open(full_path, 'r') as f:
    lines = f.readlines()
    for line in lines:
        tmp = line.split()
        read_pose = PoseStamped()
        read_pose.pose.position.x = float(tmp[0])
        read_pose.pose.position.y = float(tmp[1])
        read_pose.pose.orientation.w = 1
        self.global_path_msg.poses.append(read_pose)
```
* rospkg를 통해 패키지 경로를 가져옴
* 지정된 파일(mando_path.txt)에서 Waypoint를 읽어와 PoseStamped로 변환
* 변환된 포즈들을 Path 메시지에 추가

### 2. 전역 경로 퍼블리시
```python
rate = rospy.Rate(20)  # 20Hz
while not rospy.is_shutdown():
    self.global_path_pub.publish(self.global_path_msg)
    rate.sleep()
```
* 20Hz 주기로 /global_path에 전역 경로를 퍼블리시
* 시뮬레이션 또는 실제 차량에서 전역 경로 참조 가능
</details>

## 2) local_path_pub.py - 로컬 경로 생성 및 퍼블리시 노드
|항목|설명|
|------|------|
|구독 토픽|/odom (nav_msgs/Odometry), /global_path (nav_msgs/Path)|
|퍼블리시 토픽|/local_path (nav_msgs/Path)|
|주요 기능 요약|차량 현재 위치 기반 최근접 Waypoint 탐색 및 로컬 경로 생성|

<details> <summary> <b> 📌 local_path 코드 분석 펼쳐보기 </b></summary>

### 1. 글로벌 경로 수신 및 저장
```python
def global_Path_callback(self, msg):
    self.global_path_msg = msg
```
* /global_path 토픽에서 경로 데이터를 수신해 저장.
* 추후 차량 현재 위치 기준으로 가까운 포인트를 찾기 위해 사용.

### 2. 차량 현재 위치 수신
```python
def odom_callback(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    self.is_status = True
```
* /odom에서 차량의 현재 위치를 수신해 저장.
* 위치 정보가 수신되면, 로컬 경로 생성이 가능하도록 is_status 플래그를 True로 설정.

### 3. 최근접 웨이포인트 탐색
```python
min_dis = float('inf')
current_waypoint = -1
for i, waypoint in enumerate(self.global_path_msg.poses):
    distance = sqrt(pow(self.x - waypoint.pose.position.x, 2) + pow(self.y - waypoint.pose.position.y, 2))
    if distance < min_dis:
        min_dis = distance
        current_waypoint = i
```
* 현재 위치에서 글로벌 경로상의 모든 포인트와의 거리를 계산.
* 그 중 가장 가까운 포인트(current_waypoint)를 탐색.

### 4. 로컬 경로 생성
```python
if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
    for num in range(current_waypoint, current_waypoint + self.local_path_size):
        tmp_pose = PoseStamped()
        tmp_pose.pose.position = self.global_path_msg.poses[num].pose.position
        tmp_pose.pose.orientation.w = 1
        local_path_msg.poses.append(tmp_pose)
```
* 최근접 웨이포인트부터 50개 포인트를 잘라서 로컬 경로 구성.
* 남은 경로가 50개 미만이면 끝까지 모두 포함.

### 5. 로컬 경로 퍼블리시
```python
self.local_path_pub.publish(local_path_msg)
```
* 생성된 로컬 경로를 /local_path로 퍼블리시.
* Pure Pursuit 등 경로 추종 알고리즘에서 이 경로를 따라 주행하도록 사용.


📝 요약
* 글로벌 경로(Global Path)에서 차량 현재 위치를 기준으로 가장 가까운 지점을 찾는다.
* 최근접 위치부터 50개 포인트를 잘라 **로컬 경로(Local Path)**로 생성한다.
* 생성된 로컬 경로는 /local_path 토픽으로 주기적으로 퍼블리시되어, pure_pursuit, lattice_planner의 입력으로 사용된다.
</details>


# 3. Lidar 클러스터링을 통한 장애물 감지
LiDAR 데이터로 실시간 클러스터링을 수행하여 장애물을 감지하는 노드. DBSCAN을 사용하여 주변 객체를 감지하고 클러스터의 중심과 경계를 계산하여 시각화함.

## 클러스터링이란?
데이터 포인트들을 유사한 특성을 가진 그룹으로 나누는 비지도 학습 방법 중 하나. 이 프로젝트에서는 여러가지 클러스터링 기법 중 DBSCAN (Density-Based Spatial Clustering of Applications with Noise) 알고리즘을 사용하여 LiDAR 데이터를 기반으로 객체를 클러스터링하였다.

### 🔹 DBSCAN 알고리즘
밀집도가 높은 영역을 중심으로 군집을 형성하는 알고리즘.

eps: 같은 클러스터로 묶일 수 있는 최대 거리
min_samples: 최소한 몇 개의 포인트가 있어야 클러스터로 인정되는지 결정하는 값
이 방법은 K-Means와 달리 클러스터 개수를 미리 지정하지 않아도 되고, 노이즈를 효과적으로 제거할 수 있다는 장점이 있다.

```python
self.dbscan = DBSCAN(eps=0.3, min_samples=3)  # DBSCAN 클러스터링 파라미터 설정
```
본 대회에서는 미니 박스와 같은 작은 물체도 인식해야 하므로, 클러스터링의 민감도를 높이기 위해 eps와 min_samples 값을 상대적으로 낮게 설정하였다.

## 1) lidar_velodyne_cluster.py 
|항목|설명|
|------|------|
|구독 토픽|/velodyne_points (sensor_msgs/PointCloud2)|
|퍼블리시 토픽|/clusters (geometry_msgs/PoseArray), /cluster_distances (std_msgs/Float32MultiArray)|

### 🔹 주요 기능 요약
|기능|설명|
|------|------|
|포인트클라우드 필터링|특정 범위 내의 포인트만 추출하여 클러스터링 수행|
|DBSCAN 클러스터링|밀집도가 높은 영역을 자동으로 군집화하고 노이즈 제거|
|클러스터 중심 좌표 계산|포인트들의 평균을 이용하여 중심 좌표 산출|
|클러스터 경계(바운딩 박스) 계산|클러스터의 최소/최대 좌표를 기반으로 네모난 경계 생성|
|클러스터 경계 꼭짓점 좌표 포함|바운딩 박스의 4개 꼭짓점을 PoseArray로 변환|
|클러스터 거리 정보 퍼블리시|각 클러스터와의 거리 정보를 /cluster_distances 토픽으로 제공|

📝 개선 내용
* 초기에는 클러스터 중심 좌표만을 이용해 경로를 생성했으나, 장애물 크기에 대한 고려가 부족해 충돌 가능성이 있음을 발견. 이를 해결하기 위해 바운딩 박스를 추가하여 클러스터의 크기를 반영하고, 보다 안정적인 회피 경로를 생성할 수 있도록 개선.
* 추가로 퍼블리시한 /cluster_distances는 객체와 차량 간의 거리를 확인하는 추가적인 안전 체크 역할을 수행하는 데이터.

<details> <summary><b>📌 lidar_velodyne_cluster.py 코드 분석 펼쳐보기</b></summary>

### 1. 노드 초기화 및 주요 토픽 설정
```python
class SCANCluster:
    def __init__(self):
        # LiDAR 데이터 수신
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)

        # 클러스터 중심 및 경계 퍼블리시
        self.cluster_pub = rospy.Publisher("/clusters", PoseArray, queue_size=1)

        # 클러스터 거리 정보 퍼블리시
        self.distance_pub = rospy.Publisher("/cluster_distances", Float32MultiArray, queue_size=1)

        self.pc_np = None
        self.dbscan = DBSCAN(eps=0.3, min_samples=3)  # DBSCAN 클러스터링 설정

        # 30Hz 주기로 실행
        rate = rospy.Rate(30)  
        while not rospy.is_shutdown():
            rate.sleep()
```
* /velodyne_points 토픽에서 LiDAR 데이터를 구독
* /clusters, /cluster_distances 토픽을 생성하여  퍼블리시
* DBSCAN을 사용하여 밀집도가 높은 영역을 클러스터링

### 2. PointCloud2 데이터를 numpy 배열로 변환
```python
def pointcloud2_to_xyz(self, cloud_msg):
    point_list = []
    for point in pc2.read_points(cloud_msg, skip_nans=True):
        dist = (point[0]**2 + point[1]**2)**0.5  # 거리 계산
        # 특정 영역 내의 포인트만 필터링
        if point[0] > 0 and point[1] > -6 and point[1] < 6 and point[2] > -0.78 and point[2] < 3 and dist < 60:
            point_list.append((point[0], point[1], dist))

    point_np = np.array(point_list, np.float32)  
    return point_np  
```
* 수집한 포인트들 중 필요한 데이터들만 필터링. (필터링을 걸어주지 않으면 연산 딜레이가 너무 심함)
* x, y, 거리 데이터만 추출하여 numpy 배열로 변환

###  3.LiDAR 데이터 수신 및 클러스터 좌표 퍼블리시
```python
def callback(self, msg):
    # LiDAR 데이터를 numpy 배열로 변환
    self.pc_np = self.pointcloud2_to_xyz(msg)

    # 거리 값만 추출하여 퍼블리시
    distances = self.pc_np[:, 2]  
    distances_msg = Float32MultiArray()
    distances_msg.data = distances
    self.distance_pub.publish(distances_msg)

    # 클러스터 중심과 경계를 퍼블리시
    cluster_corners_and_center = self.cluster(self.pc_np[:, :2])
    self.cluster_pub.publish(cluster_corners_and_center)
```
 * /velodyne_points에서 데이터를 받아서 numpy 배열로 변환
 * 각 포인트의 거리 값을 Float32MultiArray로 변환하여 퍼블리시
 * 클러스터링 후 중심 좌표 및 경계를 /clusters 토픽으로 퍼블리시

### 4. 클러스터링 (DBSCAN 적용 및 중심/경계 계산)
```python
def cluster(self, xy):
    # x, y 좌표만 사용하여 DBSCAN 클러스터링 수행
    db = self.dbscan.fit_predict(xy)  
    n_cluster = np.max(db) + 1  

    cluster_msg = PoseArray()  
    cluster_msg.header.frame_id = "/map"
    cluster_msg.header.stamp = rospy.Time.now()

    # 각 클러스터에 대해 경계 및 중심 계산
    for cluster in range(n_cluster):
        cluster_points = xy[db == cluster, :]

        if len(cluster_points) > 0:
            # 최소/최대 좌표 계산 (경계 박스)
            x_min, y_min = np.min(cluster_points, axis=0)
            x_max, y_max = np.max(cluster_points, axis=0)

            # 클러스터 중심 좌표 계산
            center_x = np.mean(cluster_points[:, 0])  
            center_y = np.mean(cluster_points[:, 1])  

            # 중심 좌표를 Pose로 변환하여 추가
            center_pose = Pose()
            center_pose.position.x = center_x
            center_pose.position.y = center_y
            center_pose.position.z = 0  
            cluster_msg.poses.append(center_pose)

            # 바운딩 박스의 4개 꼭짓점 계산 및 추가
            corners = np.array([
                [x_min, y_min], [x_min, y_max], [x_max, y_min], [x_max, y_max]
            ])

            for corner in corners:
                pose = Pose()
                pose.position.x = corner[0]
                pose.position.y = corner[1]
                pose.position.z = 0  
                cluster_msg.poses.append(pose)

    return cluster_msg
```
* DBSCAN을 적용하여 클러스터를 자동으로 생성
* 클러스터 중심 좌표를 계산하여 Pose로 변환 후 추가
* 클러스터 경계를 계산하고 4개의 꼭짓점을 Pose로 변환하여 추가로 포함 시킴

</details>

## 2) lidar_velodyne_cluster_viz.py - LiDAR 클러스터 시각화 노드
|항목|설명|
|------|------|
|구독 토픽|/clusters (geometry_msgs/PoseArray)|
|퍼블리시 토픽|/visualization_marker_array (visualization_msgs/MarkerArray)|
|주요 기능 요약|/clusters 토픽의 데이터를 받아 전역좌표계로 변환하고 RViz에서 시각적으로 표현|

<details> <summary> <b> 📌 lidar_velodyne_cluster_viz.py 코드 분석 펼쳐보기 </b></summary>

### 1. 노드 초기화
```python
class Cluster_viz:
    def __init__(self):
        rospy.Subscriber("/clusters", PoseArray, self.callback)  # 클러스터 데이터 구독
        rospy.Subscriber("/odom", Odometry, self.odom_callback)  # 차량 위치 및 자세 구독
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.status_callback)  # 차량 속도 구독

        self.object_pointcloud_pub = rospy.Publisher('object_pointcloud_data', PointCloud, queue_size=1)  # 객체 데이터 퍼블리시

        self.is_odom = False
        self.cluster_status = False
        self.dangerous_status = False
        self.prev_dangerous_data = None
        self.cluster_data = None  # 초기화

        rate = rospy.Rate(50)  # 50Hz 주기 실행
        while not rospy.is_shutdown():
            if self.is_odom and self.cluster_status:
                # (1) 차량의 방향을 반영한 좌표 변환 행렬 생성
                trans_matrix = self.trans_matrix(self.vehicle_yaw)

                # (2) 클러스터 데이터를 전역 좌표계로 변환
                obj_data_cluster = self.tf_global(trans_matrix, self.cluster_data)

                # (3) 변환된 객체 데이터를 퍼블리시
                self.object_pointcloud_pub.publish(obj_data_cluster)

            rate.sleep()
```
* /clusters, /odom, /Competition_topic 3가지 주요 데이터를 구독
* /object_pointcloud_data 토픽을 생성하여 변환된 객체 데이터를 퍼블리시
* 차량의 현재 위치와 방향을 고려하여 LiDAR 데이터를 전역 좌표로 변환

### 2.차량의 방향을 반영한 좌표 변환 행렬 생성
```python
def trans_matrix(self, vehicle_yaw):
    trans_matrix = np.array([[math.cos(vehicle_yaw), -math.sin(vehicle_yaw), 0],
                             [math.sin(vehicle_yaw), math.cos(vehicle_yaw), 0],
                             [0, 0, 1]], dtype=np.float32)
    return trans_matrix
```
* 차량의 회전(yaw)을 반영하는 변환 행렬을 생성
* 차량의 현재 방향을 고려하여 LiDAR 데이터를 전역 좌표계로 변환

### 3. 클러스터 데이터를 전역 좌표계로 변환
```python
def tf_global(self, trans_matrix, cluster_data):
    obj_data = PointCloud()
    obj_data.header.frame_id = 'map'
    obj_data.header.stamp = rospy.Time.now()  # 타임스탬프 추가
    obj_data.points = []  # 포인트 리스트 초기화

    vehicle_pos_x = self.vehicle_pos_x  # 현재 차량의 x 위치
    vehicle_pos_y = self.vehicle_pos_y  # 현재 차량의 y 위치

    for num, i in enumerate(cluster_data.poses):
        # (2) 로컬 좌표를 전역 좌표로 변환
        local_result = [i.position.x + 1.5, i.position.y, 1]  # 차량 기준으로 x 축 1.5m 이동
        temp = trans_matrix.dot(local_result)  # 변환 행렬 적용
        global_result = [temp[0] + vehicle_pos_x, temp[1] + vehicle_pos_y]

        # (3) 전역 좌표를 PointCloud에 추가
        tmp_point = Point32()
        tmp_point.x = global_result[0]
        tmp_point.y = global_result[1]
        tmp_point.z = 1  # 필요에 따라 수정 가능
        obj_data.points.append(tmp_point)

    return obj_data
```
* 클러스터 데이터를 차량 좌표에서 전역 좌표로 변환
* 변환 행렬을 적용하여 차량 방향을 반영
* 변환된 좌표를 PointCloud 포인트 리스트에 추가

### 4. 클러스터 데이터 및 차량의 상태 (속도, 위치 등) 데이터 수신
```python
 def callback(self, msg):    
        self.cluster_data = msg
        self.cluster_status = True

    def status_callback(self, msg):  # 속도만 처리
        self.is_status = True
        self.vehicle_velocity = msg.velocity.x * 3.6  # 속도를 km/h로 변환

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.vehicle_pos_x = msg.pose.pose.position.x
        self.vehicle_pos_y = msg.pose.pose.position.y
```
</details>

# 4. 정적 회피 주행 알고리즘

## 1. 경로 추종 - Pure pursuit guidance

<table>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/6108b268-51f4-4947-8cff-6f5791252358" alt="Pure Pursuit Diagram" width="500">
    </td>
    <td> 
     - L : Wheel base <br>
     - δ : Steering <br>
     - α : Look-ahead point angle <br>
     - R : Curvature radius <br>
     - dla: Look-ahead distance <br>
     - vx: Longitudinal speed
    </td>
  </tr>
</table>

* 경로 위의 한 점을 원 호를 그리며 추종하는 알고리즘  
* 자동차의 기구학과 전방주시거리 (Look-Ahead-Distance)라는 하나의 파라미터를 사용하여 조향각을 계산  
* 차량의 후륜 축 중심을 기준으로 계산

⚠ 단순히 pure pursuit만으로는 목표 속도에 도달하는 시간이 느리고, 오차가 심하므로 이 것만으로는 주행에 사용할 수 없음!

### 1) 종방향 PID 제어기 설계 및 추가 
PID (Proportional-Integral-Derivative) 제어
<table>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/1beee2e8-61bc-4cbd-8127-ca732e3e1173" alt="이미지 설명" width="400">
    </td>
    <td> 
      <img src="https://github.com/user-attachments/assets/d521c2ad-e31a-4ff6-a874-c3656ade971a" alt="이미지 설명" width="300">
    </td>
  </tr>
</table>

* 원하는 값에 도달하기 위한 기초적인 피드백 제어 방법 중 하나
* P, PI, PD, PID 등 제어 대상에 맞게 선택해서 사용할 수 있음
* PID 제어는 수식이 매우 간단하고, 구현 난이도 대비 목표치 추종이나 외란 감쇄 효과에 탁월한 성능을 가짐
* 적절한 이득값 조절이 필요 (K_p,K_I,K_D)

✅ 실제 구현 결과 목표 속도에 더욱 빠르게 도달하고, 오차가 적은 것을 확인할 수 있었음!

<details> <summary><b>📌 pidControl class 코드 분석 펼쳐보기</b></summary>

### 1. pidControl 클래스 개요
```python
class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02
```
* ✅ 이 클래스는 차량의 속도를 PID 제어를 통해 조정하는 역할을 수행
* ✅ p_gain, i_gain, d_gain 값을 조절하여 차량이 목표 속도에 정확하게 도달하도록 조작

* p_gain → 현재 오차에 비례하여 보정
* i_gain → 과거 오차를 누적하여 보정
* d_gain → 오차 변화율(속도 차이 변화)을 고려하여 보정
* controlTime → 제어 주기(샘플링 간격), 20ms (0.02초)로 설정

### 2. PID 연산 과정
```python
def pid(self, target_vel, current_vel):
    error = target_vel - current_vel
    p_control = self.p_gain * error
    self.i_control += error * self.controlTime  
    d_control = self.d_gain * (error - self.prev_error) / self.controlTime
    output = p_control + self.i_gain * self.i_control + d_control
    self.prev_error = error
    return output
```

| **제어 항목** | **역할** | **공식** |
|--------------|---------|---------|
| **P (비례 제어)** | 현재 오차 반영 | `P_out = K_p * e(t)` |
| **I (적분 제어)** | 과거 오차 누적 보정 | `I_out = K_i * Σ e(t) * Δt` |
| **D (미분 제어)** | 오차 변화율 보정 | `D_out = K_d * (Δe(t) / Δt)` |
| **최종 PID 출력** | 위 3가지 합산 | `output = P_out + I_out + D_out` |

### 3. 종방향 제어(Pure Pursuit + PID) 적용 흐름
PID 제어기는 Pure Pursuit과 함께 사용되어 목표 속도에 빠르게 도달하면서,
속도 오차를 줄여 주행 안정성을 향상시킨다.

### 🚗 [Pure Pursuit + PID 속도 제어 흐름]

1. 차량의 현재 속도(current_vel)와 목표 속도(target_vel) 차이를 계산
2. PID 연산을 수행하여 가속(Throttle) 또는 감속(Brake) 값을 결정
3. PID 제어값(output)이 0보다 크면 가속, 0보다 작으면 감속
4.  차량이 목표 속도에 안정적으로 도달  

```python
# PID 출력값을 Throttle/Brake로 변환
output = pidControl().pid(target_velocity, current_velocity)

if output > 0:
    ctrl_cmd_msg.accel = output  # 가속
    ctrl_cmd_msg.brake = 0.0
else:
    ctrl_cmd_msg.accel = 0.0
    ctrl_cmd_msg.brake = -output  # 감속
```
* PID 제어가 없을 경우 → 속도 도달 시간이 길고 오차가 큼
* PID 제어를 추가하면 → 목표 속도에 빠르게 도달하고 안정적인 주행 가능

#### 📌 결론

* Pure Pursuit 단독으로는 속도 오차가 크므로, PID 제어기를 추가하여 속도 추종 성능을 향상
* PID 제어기는 P(비례) / I(적분) / D(미분) 항목을 조합하여 최적의 속도 조절 수행
* 목표 속도에 더 빠르게 도달하며, 제어 성능이 개선되었음

✅ 결과적으로, Pure Pursuit과 PID 제어기의 조합은 기존보다 안정적인 경로 추종을 가능하게 함!
</details>

### 2) 경로 기반 속도 계획 추가
* 차량 외부 요인 중 도로 모양만을 고려하여 속도 계획
* 곡률을 구해서 곡률 기반에서 차량이 주행할 수 있는 최대 속도를 계산
* 차량은 곡선에서 주행할 때 원심력을 받기 때문에, 곡선에서는 속도를 낮춰야 함. 따라서 속도 계획을 통해 차량이 전복되는 것을 방지하고, 직선에서는 최대 속도로 주행하며, 곡선에서는 안정적인 속도로 주행 가능

#### 🚨 기존 문제점: Pure Pursuit만 사용했을 때의 한계
Pure Pursuit 알고리즘은 차량이 경로를 따라가는 데 효과적이지만, 속도 조절 기능이 없어 커브 구간에서 매우 불안정함.

* 직선 구간에서는 빠르게 주행 가능하지만 곡선 구간에서도 속도가 일정해버리면 차량이 커브를 돌 때 크게 벗어날 수 있음.
* 속도를 조절하지 않으면 언더스티어(Understeer) 또는 오버스티어(Oversteer) 발생 가능
  
✅ 해결 방법: 도로의 곡률 기반 속도 계획 추가

이 문제를 해결하기 위해 경로 기반 속도 계획(Velocity Planning)을 추가하여, 곡률 반경을 계산하고 적절한 속도를 설정함.

* 도로의 곡률 반경을 계산하여 회전 반경이 작은 경우 속도를 줄임
* 도로가 직선에 가까울 경우 차량 최대 속도로 주행
* 도로 마찰 계수(노면 상태)까지 고려하여 현실적인 속도 조절
  
#### 동작 요약

1️⃣ 경로를 분석하여 도로의 곡률 반경(r)을 계산

2️⃣ 최소 제곱법(Least Squares Method, LSM)을 사용해 곡률 반경을 구함

3️⃣ 곡률 반경과 도로 마찰 계수를 활용하여 속도 제한을 설정

4️⃣ 커브에서는 감속, 직선에서는 가속 → 안정적인 주행 가능!


<details> <summary><b>📌 velocityPlanning 상세 분석 펼쳐보기</b></summary>
  
### 📌 ① 최소자승법(Least Squares Method)을 이용한 곡률 반경 계산
  
```python
class velocityPlanning:
    def __init__ (self, car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            #TODO: (6) 도로의 곡률 계산 - 최소 자승법 이용하여 곡률 반지름 "r" 계산
            #========================================================#
            A = np.array(x_list)
            B = np.array(y_list)
            a, b, c = np.dot(np.linalg.pinv(A), B)  

            r = (a**2 + b**2 - c)**0.5       
```

- 경로의 초반 구간은 기준이 되는 앞뒤 포인트 수가 부족하므로 곡률 계산이 어렵다. 따라서 최대 속도를 그대로 사용하였다.
  
- 원의 방정식 $`(x - a)^2 + (y - b)^2 = r^2`$ 을 전개하여 정리하면, $`-2ax - 2by + c = -x^2 - y^2`$처럼 선형 형태로 나타낼 수 있다.
  
- 이를 최소자승법 형태인 $`A · [a, b, c]^T = B`$ 로 구성하여, 중심 좌표 $`(a, b)`$와 $`r^2 = a^2 + b^2 - c`$를 계산한다.
  
- 이때, $`[a, b, c]^T = (A^T A)^{-1} A^T B = A^+ B`$ 를 통해 최소자승 해를 구할 수 있으며 Python에서는 `np.linalg.pinv()`를 사용하여 의사역행렬(Pseudo-inverse) 방식으로 안정적인 해를 계산할 수 있다.

- 계산된 곡률 반경 `r`이 **작을수록 커브가 심한 구간**, **클수록 직선에 가까운 구간**임을 의미한다.


### 📌 ② 곡률 기반 최고 속도 계산
```python
v_max = sqrt(r * 9.8 * self.road_friction)
if v_max > self.car_max_speed:
    v_max = self.car_max_speed
out_vel_plan.append(v_max)
```
-  $`v_max = sqrt(r*g*𝜇)`$ 공식을 기반으로 하여 최고 속도를 계산한다.
-  곡률이 클수록 (직선일수록) 높은 속도를 허용하고, 곡률이 작을수록 (급커브) 낮은 속도로 제한한다.
-  최대 속도 제한 고려하여 v_max를 클램핑(clamping)


</details>


# 2. 충돌 회피 알고리즘 - State Lattice Planning

## 1) 개요 및 이론적 배경
<p align="left">
  <img src="https://github.com/user-attachments/assets/edb0861d-205d-4cee-8170-948b4d137a32" alt="이미지 설명" width="600">
</p>

State Lattice Planner는 차량이 실제로 움직일 수 있는 경로만 생성해주는 경로 계획 방법이다. 보통 경로를 만들 때는 그냥 직선이나 곡선을 기준으로 단순하게 만드는 경우가 많지만, 그렇게 되면 차량이 물리적으로 따라갈 수 없는 경로가 생길 수 있다.

예를 들어, 사람이 길을 갈 때 계단 위로 똑바로 점프해서 올라가는 건 불가능하듯이, 차량도 급격하게 방향을 틀거나 좁은 공간을 빠르게 도는 건 힘들다. 그래서 차량의 특성(조향 각도, 회전 반경 등)을 고려해서, 처음부터 따라갈 수 있는 경로만 미리 만들어 놓는 것이 바로 State Lattice 방식이다.

이 방식에서는 차량이 앞으로 가는 여러 방향에 대해 가능한 경로(회피 경로)를 미리 계산해두고, 장애물이나 도로 정보와 비교해서 가장 안전하고 현실적인 경로를 선택하게 된다.

### ✔ 실시간 회피 경로 생성
State Lattice Planner는 자율주행 차량이 전방 장애물을 인식했을 때, 회피 가능한 여러 개의 후보 경로를 실시간으로 생성하고, 이 중 가장 안정적이고 충돌 가능성이 낮은 경로를 선택하여 주행하게 한다. 이는 특히 **복잡한 도심 환경이나, 도로 구조가 불완전한 경우(예: 인도 경계가 불분명한 상황)**에서 유용하게 활용된다.

## 2) 프로젝트 내 적용 맥락
본 프로젝트에서는 MORAI 시뮬레이터 상에서 정적 장애물을 회피하기 위한 용도로 State Lattice Planner를 구현하였으며, 다음과 같은 구조로 적용되었다:

입력 정보: `/local_path` (차량 중심 기준의 전방 경로), `/object_pointcloud_data` (장애물 클러스터링 결과), `/right_path` (우측 경계선 정보), `/odom`, `/velocity`

출력 정보: `/lattice_path` (선택된 최종 회피 경로), `/lattice_path_n` (시각화를 위한 후보 경로), `/speed_control` (정지 신호 여부)

차량 전방에 대해 15개의 회피 후보 경로를 생성하고, 각 경로가 장애물 및 도로 경계와 얼마나 가까운지 등을 판단하여 **가중치 기반 평가(weighted evaluation)**를 수행한다.
최종적으로 가장 안전하고 도로 구조를 잘 따르는 경로가 `/lattice_path`로 선택되어 차량이 이를 추종하게 된다.

## 3) 기존 한계점
기본 Lattice Planner는 장애물 유무만을 고려한 단순한 충돌 체크 기반으로 작동하기 때문에, 다음과 같은 문제가 발생하였다:

⚠ 도로 경계 인식 불가: 인도 또는 차선 바깥으로 경로가 생성되어 차량이 도로 밖으로 이탈하는 문제 발생

⚠ 경로 정밀도 부족: 객체와의 단순 거리 기반 회피로 인해, 실제 회피 가능한 경로를 선택하지 못하고 반복적으로 잘못된 경로가 선택되는 경우 존재

⚠ 경로 평가 기준 미비: 경로 선택 시 도로 구조나 방향성이 고려되지 않아 비현실적인 경로가 선택될 가능성 있음

## 3) 개선 사항
### ✅ 1. 도로 경계 (우측 경로)와의 충돌 판단 추가
- 기존에는 장애물과의 거리만 고려했으나, right_path (도로 가장자리 정보)를 따로 받아와 차량이 인도 등 비정상 영역으로 벗어나지 않도록 경로를 필터링함.

- 차량이 우측 경계와 너무 가까워지면 (dis < 1), 해당 경로는 충돌 위험이 있다고 판단하여 제외.

### ✅ 2. 객체와의 거리 기반 판단 강화
- `/cluster_distances` 토픽을 활용해 객체와의 직접 거리 정보를 받아옴.

- 특정 거리(예: 4.5m) 이내로 접근 시, 객체가 너무 가까운 것으로 판단하여 회피 경로를 생성하도록 유도.

### ✅ 3. 차선 기준 경로 분류 및 가중치 부여
- 차량이 1차선에 있는지 아닌지를 right_path 기준으로 판단한 후, 그에 따라 불필요한 경로에 높은 가중치를 부여함.

- 예: 1차선이라면 도로 바깥쪽(인도 방향)으로 빠지는 경로를 억제.

### ✅ 4. 외적(Cross Product) 기반 경로 방향성 판단
- 경로 방향성과 도로 경계선의 진행 방향이 일치하는지 확인하기 위해 벡터 외적을 사용함.
- 외적 결과에 따라 경로가 도로 안쪽/바깥쪽을 향하는지 판단할 수 있음.

### 🔹 외적 판단 예시
```python
노란선 (우측 경계선) 방향: →→→
회피 경로1: ↘ (밖으로 나감 → 외적 음수 → 가중치 ↑)
회피 경로2: → (같은 방향 → 외적 0 → 괜찮음)
회피 경로3: ↗ (안쪽으로 들어감 → 외적 양수 → 괜찮거나 더 좋음)
```
즉, 외적 값이 음수면, 그 경로는 도로에서 바깥쪽으로 벗어날 가능성이 있으니까 가중치를 높여서 선택되지 않도록 만듬.

### ⚠️ 외적 기반 경로 필터링의 좌표계 의존성 문제
하지만 이러한 외적 필터는 다음과 같은 한계가 존재했다:

- 본 프로젝트에서 사용한 트랙은 직선이 아닌 직사각형 형태로 구성되어 있음.
- 이로 인해 차량이 직진 주행 중이라도 구간에 따라 방향이 +x, -x, +y, -y로 변화하게 됨.
- 그러나 외적(cross product)을 통한 방향 비교는 전역 좌표계 기준으로 계산되기 때문에, 특정 구간에서는 실제로 도로 안쪽으로 향하는 회피 경로도 외적 값이 음수로 나와 잘못된 경로로 필터링될 수 있음.
- 이는 외적 기반 경로 판단이 **도로의 국소적 진행 방향을 고려하지 않고** 전역 좌표 기준으로 수행되었기 때문에 발생한 구조적 한계임.

✅ 이 문제는 국소 좌표계 기반 판단(예: Frenet frame)으로 보완 가능할 것으로 예상


### ✅ 5. 도로 곡률(Radius of Curvature) 반영
- 경로의 곡률 반경을 최소자승법(LSM)을 통해 계산하여, 커브길 등에서 더 보수적인 회피 경로를 유도함.

- 곡률이 작을수록 (급커브) → 더 우측 경로 선택에 민감하게 작동되도록 외적 조건을 강화.
  
- ex) 곡률 반경(`min_r`)이 작다 → 커브 구간 → 더 많은 회피 경로(0~7번)에 대해 도로 방향과 외적 판단을 적용해 우측 이탈 경로를 철저히 제거
  
  곡률 반경이 크다 → 직선 구간 → 회피 경로가 바깥쪽으로 살짝 벗어나도 위험하지 않기 때문에 외적 필터링 범위를 줄임 (0~4번)

### ✅ 6. 지정 영역(사각형) 침범 방지
- 특정 도로 영역(예: 인도 영역)을 사각형으로 설정하고, 경로가 그 내부에 진입할 경우 불이익을 주도록 개선.

- 이는 경로가 실제 도로를 벗어나는 것을 방지하기 위한 보조 필터링 로직임.


이러한 개선을 통해 차량이 도로 바깥으로 벗어나는 문제를 해결하였고, 실제 MORAI 시뮬레이터에서는 도로 중심을 따라 안정적으로 장애물을 회피 주행하며, 대회 참가 팀 중 몇 안 되는 **완주에 성공하는 성과**를 거둘 수 있었다.










