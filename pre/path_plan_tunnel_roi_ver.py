#!/usr/bin/env python3
# # -*- coding: utf-8 -*-

from turtle import right
import rospy
# import tf
import os
from std_msgs.msg import Float32MultiArray, String
# from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point
from pyproj import Proj
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import distance
import pyproj
from math import pi,sqrt
import math
from geometry_msgs.msg import Quaternion
from pyproj import CRS, Transformer
from geometry_msgs.msg import PoseArray, Pose
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
# from vehicle_msgs.msg import TrackCone, Track
import numpy as np
import time
from std_msgs.msg import Int32, Bool,String
from visualization_msgs.msg import MarkerArray, Marker


class GPS2UTM:
    def __init__(self):
        rospy.loginfo("GPS2UTM is Created")

        # ------------------------- Subscriber ----------------------
        rospy.Subscriber("/adaptive_clustering/markers", MarkerArray, self.lidar_callback)
        rospy.Subscriber("/current_waypoint", Int32, self.waypoint_callback)

        # -------------------------- Marker ----------------------
        self.middle_point_pub = rospy.Publisher("target_point", Marker, queue_size=10)
        self.left_point_pub = rospy.Publisher("left_point", Marker, queue_size=10)
        self.right_point_pub = rospy.Publisher("right_point", Marker, queue_size=10)
        # ------------------------- Publisher ----------------------------
        self.target_point_publisher = rospy.Publisher("avoid_point", Float32MultiArray, queue_size=10) #터널 안 정적 장애물 회피시 사용
        self.obstacle_state_pub = rospy.Publisher('obstacle_state', String, queue_size=5)
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)

        # 동적 장애물 인지 시 roi
        self.front_roi = [0.1, 4]
        self.side_roi = [-2.0, 2.0]

        # obstacle cnt
        self.cnt_obstacle = 0
        self.dist_obstacle = np.inf
        self.dynamic_count = 0
        self.last_obs_x = 0

        self.avoid_trigger = False
        self.return_trigger = False
        self.return_except = 0

        self.right_offset = 4 # 오른쪽 벽과의 거리 오프셋
        self.left_offset = 10 # 왼쪽 벽과의 거리 오프셋
        self.RETURN_END_THRESHOLD = 1. # 평행이 맞았다는 기준

        self.disappear_obs_count=0
        self.offset = 0.
        self.return_time = 0.0
        self.complete_Dynamic = False
        # waypoint
        self.current_waypoint = 5
        self.tunnel_waypoint = np.arange(1 , 1e6)

    def waypoint_callback(self, msg):
        self.current_waypoint = msg.data
    

    def lidar_callback(self, msg):
        # msg: visualization_msgs/MarkerArray
        obstacle_list = []

        self.current_time = time.time()

        # header는 첫 마커 기준(없으면 now)
        hdr = msg.markers[0].header if msg.markers else None
        bev_msg = PoseArray()
        if hdr:
            bev_msg.header = hdr
        else:
            bev_msg.header.stamp = rospy.Time.now()
            bev_msg.header.frame_id = "velodyne"

        tunnel_right_side = []
        tunnel_left_side  = []

        # not in tunnel, return
        if not self.current_waypoint in self.tunnel_waypoint:
            self.obstacle_state_pub.publish("Safe")
            print("NOT IN TUNNEL")
            return

        # ------------------- MarkerArray → 박스 센터/사이즈 추출 --------------------
        # 허용 타입: LINE_LIST(5), CUBE(1), CUBE_LIST(6)
        for m in msg.markers:
            if m.type not in (Marker.LINE_LIST, Marker.CUBE, Marker.CUBE_LIST):
                continue

            # 좌표계 보정은 생략(입력 frame_id="velodyne" 가정)
            if m.type == Marker.LINE_LIST:
                # 선분들의 점들로부터 AABB 생성
                if len(m.points) < 4:
                    continue
                minx = float('inf'); miny = float('inf')
                maxx = float('-inf'); maxy = float('-inf')
                for p in m.points:
                    if p.x < minx: minx = p.x
                    if p.x > maxx: maxx = p.x
                    if p.y < miny: miny = p.y
                    if p.y > maxy: maxy = p.y

                center_x = 0.5 * (minx + maxx)
                center_y = 0.5 * (miny + maxy)
                width  = max(0.0, maxx - minx)
                height = max(0.0, maxy - miny)

            else:
                # CUBE / CUBE_LIST : pose + scale 사용
                s = m.scale
                if not (s.x > 0.0 and s.y > 0.0):
                    continue
                center_x = m.pose.position.x
                center_y = m.pose.position.y
                width  = s.x
                height = s.y

            # 장애물 크기(대략) — 기존 로직 호환
            obstacle_size = math.sqrt(width**2 + height**2)
            d = distance.euclidean((center_x, center_y), (0.0, 0.0))

            # ------------------------------- tunnel roi -------------------------------
            if obstacle_size > 4.0:  # 터널로 판정
                if center_y < 0:      # y<0 → 오른쪽 벽
                    tunnel_right_side.append([center_x, center_y, d])
                elif center_y > 0:    # y>0 → 왼쪽 벽
                    tunnel_left_side.append([center_x, center_y, d])
                # 터널은 obstacle_list에 넣지 않음
                continue

            # ------------------------------- obstacle roi -----------------------------
            # 기존 필터: 크기 0.3~1.5 사이만 “장애물”
            if obstacle_size <= 1.5 and obstacle_size >= 0.3:
                if self.front_roi[0] < center_x < self.front_roi[1]:
                    if self.side_roi[0] < center_y < self.side_roi[1]:
                        obstacle_list.append([center_x, center_y, d])

        # ------------------------- tunnel process ---------------------------------
        tunnel_left_point = None
        tunnel_right_point = None

        if len(tunnel_right_side) > 0:
            tunnel_right_side.sort(key=lambda x: x[2])  # 거리 기준
            tunnel_right_point = tunnel_right_side[0]
            self.publish_obstacles(tunnel_right_point, self.right_point_pub, color=(0.0, 0.0, 1.0))

        if len(tunnel_left_side) > 0:
            tunnel_left_side.sort(key=lambda x: x[2])
            tunnel_left_point = tunnel_left_side[0]
            self.publish_obstacles(tunnel_left_point, self.left_point_pub, color=(0.0, 0.0, 1.0))

        # -------------------------------------------------------------------------------
        if len(obstacle_list) == 0:
            if self.cnt_obstacle >= 2:
                self.return_afterAvoiding(tunnel_left_point, tunnel_right_point)
                return
            elif not self.dynamic_count == 0 and not self.complete_Dynamic:
                self.disappear_obs_count += 1
                if self.disappear_obs_count > 20:
                    self.complete_Dynamic = True
                    print("동적 장애물 회피 완료. 정적 장애물 회피 시작")
                else:
                    self.obstacle_state_pub.publish("Dynamic")
                    print(f"disappear_obs_count 카운팅 중 {self.disappear_obs_count}번 ")
                    return
            else:
                print("None obstacle!!!")
                self.obstacle_state_pub.publish("Safe")
                return

        print("{}st obstacle detecting, distance : {:2f}!!".format(self.cnt_obstacle, self.dist_obstacle))
        print("avoid trigger : {}".format(self.avoid_trigger))

        obstacle_list.sort(key=lambda x: x[0])  # x(전방) 기준
        if len(obstacle_list) == 0:
            pass
        else:
            nearest_obstacle = obstacle_list[0]

        # --------------------------- 동적 장애물 분기 ---------------------------
        if self.dist_obstacle == np.inf and not self.complete_Dynamic:
            if nearest_obstacle[2] < 5 and self.dynamic_count == 0:
                print("동적 장애물과의 거리가 5m 이내, 동적 장애물 처음 처음 처음")
                self.obstacle_state_pub.publish("Dynamic")
                self.dynamic_count += 1
                self.last_obs_x = nearest_obstacle[0]
                return

            elif nearest_obstacle[2] < 5 and abs(nearest_obstacle[0] - self.last_obs_x) < 1.5:
                print("동적 장애물 나타난 후 계속 존재")
                self.obstacle_state_pub.publish("Dynamic")
                self.dynamic_count += 1
                self.last_obs_x = nearest_obstacle[2]
                return
            else:
                self.obstacle_state_pub.publish("Safe")
                print("장애물 인식은 되었지만 일정거리 내에 들어오지 않았을 때")

        # ------------------- 동적 이후 정적 장애물 회피 시작 -------------------
        if self.dist_obstacle == np.inf and self.complete_Dynamic:
            self.side_roi  = [-2.5, 2.5]
            self.front_roi = [0.2, 9]
            self.obstacle_state_pub.publish("Safe")

            if len(obstacle_list) >= 2:
                self.dist_obstacle = nearest_obstacle[2]
                self.avoid_trigger = True
                self.cnt_obstacle = 1

                secondary_obstacle = obstacle_list[1]
                diff_x = secondary_obstacle[0] - nearest_obstacle[0]
                diff_y = secondary_obstacle[1] - nearest_obstacle[1]
                gradient_obstacle = math.atan2(diff_y, diff_x)
                self.offset = -1.8 if gradient_obstacle < 0 else 1.8
            else:
                if nearest_obstacle[2] < 1.5:
                    self.dist_obstacle = nearest_obstacle[2]
                    self.avoid_trigger = True
                    self.cnt_obstacle = 1
                    self.offset = 1.8 if nearest_obstacle[1] < 0 else -1.8

        # --------------------------------- avoid static ---------------------------------
        if self.avoid_trigger:
            mid_point = None

            if self.dist_obstacle + 0.5 < nearest_obstacle[2] and self.cnt_obstacle == 0:
                self.cnt_obstacle += 1
                self.dist_obstacle = nearest_obstacle[2]
            elif self.dist_obstacle + 0.5 < nearest_obstacle[2] and self.cnt_obstacle == 1:
                self.cnt_obstacle += 1
                self.dist_obstacle = nearest_obstacle[2]
            else:
                if self.dist_obstacle > nearest_obstacle[2] + 1.0:
                    pass
                else:
                    self.dist_obstacle = nearest_obstacle[2]

            if self.cnt_obstacle == 1:
                mid_point = (nearest_obstacle[0], nearest_obstacle[1] + self.offset)
            elif self.cnt_obstacle == 2:
                self.return_time = time.time()
                self.front_roi = [0.1, 5]
                mid_point = (nearest_obstacle[0], nearest_obstacle[1] - self.offset)
            else:
                self.avoid_trigger = False

            if mid_point is not None:
                self.obstacle_state_pub.publish("Static")
                self.publish_obstacles(mid_point, self.middle_point_pub, color=(0.0, 1.0, 0.0))

                target_point = Float32MultiArray()
                target_point.data.append(mid_point[0])
                target_point.data.append(mid_point[1])
                self.target_point_publisher.publish(target_point)





    # 장애물 회피 후 원래 경로로 복귀하는 동작
    def return_afterAvoiding(self,tunnel_left_point, tunnel_right_point): #왼쪽, 오른쪽 터널 점 중 가장 가까운 점
        # 장애물만 해당된다. 터널은 roi 해당 안됨.
        self.side_roi = [-1.8, 1.8] # 측면 roi 설정 (단위 : m) -> 회피 후 원래 경로로 돌아가는 동안의 탐지 범위
        self.front_roi = [0.1, 5.] # 전방 roi 설정
        self.avoid_trigger = False
                
        return_time = time.time() - self.return_time # 경로 복귀 시간 계산
        return_point = None # return-point -> 복귀 목표 지점
        side_dir = "None" # 복귀 방향
        
        # if return_time < 6:
        #     print("Return_time : {:.2f}".format(return_time))
        #     # if return_time < 4.5 :
        #     #     return_point = [3, self.offset/2]         
        #     # else:
        #     #     return_point = [3, -self.offset/2]
        if not self.return_trigger: # 초기 return_trigger 값 : False -> 복귀 동작이 아직 되지 않은 경우에 


            if tunnel_right_point is not None: # 오른쪽 터널 점이 존재하는 경우
                return_point = [tunnel_right_point[0], tunnel_right_point[1]+self.right_offset] # 오른쪽 터널점에 offset을 더해서 return_point 계산
                side_dir = "RIGHT"

            elif tunnel_left_point is not None:
                return_point = [tunnel_left_point[0], tunnel_left_point[1]-self.left_offset] # 왼쪽 터널점에 offset을 빼서 return_point 계산
                side_dir = "LEFT"

            else: # 터널점이 없을 경우
                return_point = [3, self.offset/2] # 기본값 설정
                self.return_except += 1 # 복귀 예외 케이스 처리

            

            self.publish_obstacles(return_point, self.middle_point_pub, color=(0.0, 1.0, 0.0)) # 복귀점 rviz 표시
            print("return distance : {:.2f}".format(return_point[1])) # 복귀해야할 포인트가 좌우 방향으로 얼마나 떨어져있는지
            print(side_dir)
            
            # 복귀 완료 조건 확인
            if side_dir == "RIGHT":
                if abs(return_point[1])<self.RETURN_END_THRESHOLD: # 복귀 지점의 y 값이 임계값 안에 들어오면 복귀 완료
                    self.return_trigger = True

            elif side_dir == "LEFT": # 복귀 지점의 y 값이 임계값 안에 들어오면 복귀 완료
                if abs(return_point[1])<self.RETURN_END_THRESHOLD:
                    self.return_trigger = True

            else: # 터널점 없었을때 처리했던 예외 횟수가 10번 초과한 경우 복귀 완료로 설정

                if self.return_except > 10 :
                    self.return_trigger = True

            # 복귀 지점 y 좌표가 self.offset/2 초과한 경우, y 좌표 조정
            if abs(return_point[1])>abs(self.offset/2):
                return_point[1] = self.offset/2

            target_point=Float32MultiArray()
            target_point.data.append(return_point[0])
            target_point.data.append(return_point[1])
            self.target_point_publisher.publish(target_point) # 복귀지점 x,y 값 발행
            
            
            self.obstacle_state_pub.publish("Return")

        else :
            print("NONE OBSTACLE")
            self.cnt_obstacle = 3
            self.obstacle_state_pub.publish("Safe")

            

    def publish_obstacles_to_array(self,point, marker_array, color=(0.0, 0.0, 1.0), marker_id=0):
        marker = Marker()
        marker.header.frame_id = "velodyne"  # 프레임 ID 설정
        marker.type = marker.SPHERE  # 마커 형태
        marker.action = marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.id = marker_id  # 마커 ID 설정
        
        marker_array.markers.append(marker)  # MarkerArray에 마커 추가
    
    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x
        y=delta_y

        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)
        
        return obs_angle, obs_dist
    

    
    def publish_obstacles(self, obs, publisher, color):
        if obs is not None:
            x, y = obs[0],obs[1]
            # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
            marker = Marker()
            marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0  # 입체적으로 보이기 위해 z 좌표를 0 이상으로 설정
            marker.scale.x = 0.6  # 포인트 크기
            marker.scale.y = 0.6
            marker.scale.z = 0.6
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            publisher.publish(marker)



    def calculate_bounding_box_center(self, bev_coords):
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y

    def calculate_bounding_box_dimensions(self, bev_coords):
        width = math.sqrt((bev_coords[2] - bev_coords[0]) ** 2 + (bev_coords[3] - bev_coords[1]) ** 2)
        height = math.sqrt((bev_coords[4] - bev_coords[2]) ** 2 + (bev_coords[5] - bev_coords[3]) ** 2)
        return width, height

    def calculate_angle_with_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        angle_rad = math.atan2(center_y - vehicle_y, center_x - vehicle_x)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    def calculate_distance_to_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        distance = math.sqrt((center_x - vehicle_x) ** 2 + (center_y - vehicle_y) ** 2)
        return distance


def run():
    rospy.init_node("gps2utm")
    new_classs= GPS2UTM()
    rospy.spin()
    

if __name__ == '__main__':
    run()

