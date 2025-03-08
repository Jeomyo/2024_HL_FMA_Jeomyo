# HL MORAI 자율주행 대회

## 📌 패키지 상세 구성 및 주요 기능
<p align="left">
  <img src="https://github.com/user-attachments/assets/a353038b-8bd2-40bd-abc0-087ef68d2ef9" alt="이미지 설명" width="600">
</p>



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


# 3. LiDAR 데이터 클러스터링
LiDAR 데이터를 활용하여 실시간 클러스터링을 수행하는 노드. DBSCAN을 사용하여 주변 객체를 감지하고 클러스터의 중심과 경계를 계산하여 시각화함.

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







