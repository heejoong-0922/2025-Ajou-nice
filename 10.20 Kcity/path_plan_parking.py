#!/usr/bin/env python3

import math
import numpy as np
from collections import deque
import matplotlib.pyplot as plt

import rospy, rospkg
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32, Bool, String, Float32

from tracking_msg.msg import TrackingObjectArray
from erp_driver.msg import erpStatusMsg


# 해야 하는 것
# 1. 각 주차 path의 시작점에서 도착 칸에 설치되는 기준 콘까지의 Local x 좌표(=LD_X) 입력
# 2. ROI 영역의 크기를 결정하는 HALF_ROI 변수 값 설정

PLAN = "Final"
# PLAN = "Pre"

LD_X = 5.0 if PLAN == "Pre" else 2.5
HALF_ROI = 0.8
BET_CONE = 2.9 if PLAN == "Pre" else 5.0

class Parking_path_gen:
    
    def __init__(self):
        plt.ion()
        self.Path_state = "Parking_path"
        self.gear = 0
        self.prev_idx = 0
        self.obstacle_detected_streak, self.is_parking_lot_streak = 0, 0
        self.is_PathState = False
        self.is_status = False
        self.is_path = False
        self.is_goal = False            # 주차 목표점 생성 여부
        self.idx_calculated = False
        self.parking_end = False
        self.is_odom = False
        self.parking_gear = False       # 전진으로 시작
        self.min_y_length = float('-inf')
        self.local_path = Path()
        self.start_points, self.parking_paths, self.objects = [], [], []
        self.parking_state = 0
        self.plot = True
        self.stopped = False
        self.flaged = False
        self.lx, self.ly = [], []           # 미리 빈 리스트로 초기화
        self.paths_loaded = False           # 플래그 추가
        self.x, self.y = 0.0, 0.0
        
        '''
        인덱스 크기 : end3 << ... << start1
        
        위에서 아래 방향으로 후진
        ⬇️
        ⬇️
        ⬇️
            ---------
            end3    |
                    |
                    |
            start3  |
            ---------
            end2    |
                    |
                    |
            start2  |
            ---------           
            end1    |
                    |
                    |
            start1  |
            ---------   
        '''
        self.go_to_back = 5  # 사선 주차 길이 -2 보다 작아야 함
        start1, end1 = 0, 0
        start2, end2 = 0, 0
        start3, end3 = 0, 0
        self.parking_idxs = [[start1, end1], [start2, end2], [start3, end3]]
        self.cut = 1
        
        
        rospy.init_node("parking_path_gen", anonymous=False)
        rospy.Subscriber('/path_state', String, self.pathState_callback)  # planner에서 "Parking_path" 로 받아옴
        rospy.Subscriber('/Y2L_fusion/fused_3d_box', TrackingObjectArray, self.object_callback, queue_size=1)
        rospy.Subscriber('/local_path', Path, self.localPath_callback)
        rospy.Subscriber('/odom_gps', Odometry, self.odom_callback)
        rospy.Subscriber('/erp42_status', erpStatusMsg, self.status_callback)
        
        
        self.end_parking_publisher = rospy.Publisher('/parking_end', Bool, queue_size = 1)
        self.parking_gear_publisher = rospy.Publisher('/parking_gear', Bool, queue_size=1)
        self.parking_path_publisher = rospy.Publisher('/parking_path', Path, queue_size=10) ## parking path
        self.parking_velocity = rospy.Publisher('/parking_velocity', Int32, queue_size=1) ## parking velocity
        self.Desired_brake_pub = rospy.Publisher('/desired_brake', Int32, queue_size=1)
        self.viz_array_pub = rospy.Publisher('/parking_viz_array', MarkerArray, queue_size=10)
        
        
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown() :
        
            if self.Path_state == "Parking_path" and self.is_status and self.is_path and self.is_odom:
                
                if self.is_goal and not self.parking_end:
                    self.target_on()  # 최종 path 생성, path publish
                        
                else:
                    if self.parking_end:
                        self.parking_path_publisher.publish(self.local_path)
                        self.go()
                        rospy.loginfo_throttle(1.0, "publishing local path...")

                    else:
                        self.handle_parking_decision()  # 주차 공간 탐색, 목적지 결정
                        
            else:
                rospy.loginfo_throttle(1.0, f"Not parking state or Not ready to drive: {self.Path_state} {self.is_status} {self.is_path} {self.is_odom}")

            self.rate.sleep()


    def localPath_callback(self, msg):
        self.is_path = True
        self.local_path = msg      
        self.lx = [ps.pose.position.x for ps in self.local_path.poses]
        self.ly = [ps.pose.position.y for ps in self.local_path.poses] 
        
        if not self.paths_loaded and len(self.lx) >= 2:
            self.loadnplot(rospkg.RosPack().get_path('gpsimu'))
            self.paths_loaded = True    
        
    def pathState_callback(self, msg):
        self.is_PathState=True
        self.Path_state = msg.data
    
    
    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg
        self.velocity = self.erpStatus_msg.speed
        self.gear = self.erpStatus_msg.gear
    
    
    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        
    def object_callback(self, msg : TrackingObjectArray):
        if not self.is_goal and not self.parking_end:
            self.objects = []
            for obj in msg.array:
                if not hasattr(obj, "bbox") or not hasattr(obj.bbox, "points") or len(obj.bbox.points) < 8: continue
                color = getattr(obj, "type_id", None)  # 11/12/13이 콘
                if color not in (11, 12, 13): continue
                if not hasattr(obj, "point"): continue
                self.objects.append((obj.point.x, obj.point.y))  # // self.objects에 콘만 걸러서 넣기
            
            
            if self.objects:
                self.min_y_length = max(self.objects, key=lambda x:x[1])[1]
    
    
    def apply_brake(self):
        int_msg = Int32()
        int_msg.data = 200
        self.Desired_brake_pub.publish(int_msg)
        
        
    def remove_brake(self):
        int_msg = Int32()
        int_msg.data = 0
        self.Desired_brake_pub.publish(int_msg)
    
    
    def find_closest_idx(self, path_x, path_y, from_x = None, from_y = None):
        xs = np.array(path_x)
        ys = np.array(path_y)
        
        if from_x:
            distances = np.hypot(xs - from_x, ys - from_y)
        else:
            distances = np.hypot(xs - self.x, ys - self.y)
            
        return int(np.argmin(distances))
             
            
    def stop(self):
        int_msg = Int32()
        int_msg.data = 0
        self.parking_velocity.publish(int_msg)
        self.apply_brake()
        
    
    def go(self):
        self.remove_brake()
        int_msg = Int32()
        int_msg.data = 3
        if PLAN == "Final" and self.is_goal:
            if len(self.gear_changing_idxs) == 2 or len(self.gear_changing_idxs) == 1:
                rospy.logwarn(f"속도:{self.velocity}, 2에 맞춤")
                int_msg.data = 2

        self.parking_velocity.publish(int_msg)
        self.parking_gear_publisher.publish(self.parking_gear)
        

    def publish_path(self, path_x, path_y, start, end):
        
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = rospy.Time.now()
        
        path_x = path_x[start:end]
        path_y = path_y[start:end]
        
        for i in range(len(path_x)):
            pose = PoseStamped()
            pose.pose.position.x = path_x[i]
            pose.pose.position.y = path_y[i]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0  # 회전 정보가 필요하지 않으므로 기본값으로 사용
            path_msg.poses.append(pose)
            
        self.parking_path_publisher.publish(path_msg)
        

    def load_path(self, full_path):
        x_list, y_list = [], []
        with open(full_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                tmp = line.split()
                x_list.append(float(tmp[0]))
                y_list.append(float(tmp[1]))
        
        return x_list, y_list


    def cur_picked_path_vis(self, state):  # 멈춰야 하는 점(주차path 시작점), 멈추기 시작한 점, 실제 멈춘 점 plot

        # === 간단 Plot ===
        plt.figure(figsize=(6, 6))
        # 기준 좌표 (빨간 점)
        if state == "starting":
            stop_x_list, stop_y_list = self.lx[self.start_points[self.parking_state]], self.ly[self.start_points[self.parking_state]]
            
        elif state == "returning":
            stop_x_list, stop_y_list = self.x_list[self.cur_idx - 2], self.y_list[self.cur_idx - 2]
            
        plt.scatter(stop_x_list, stop_y_list,
                        c='red', s=100, marker='X', label='Target Stop (line)')
            # stop() 호출 순간 좌표 (파란 점)
        plt.scatter(self.flaged_x, self.flaged_y,
                        c='blue', s=100, marker='o', label='Flaged Stop')
            
        # 실제 멈춘 좌표 (초록 점)
        plt.scatter(self.x, self.y,
                    c='green', s=100, marker='*', label='Stopped Now')
        # local path
        self.plot_local_path()
    
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Stop Position Check")
        plt.axis("equal")
        plt.legend()
        plt.grid(True)
        plt.show(block=False)
        plt.pause(0.001)
    
    
    def plot_local_path(self):
        
        if self.lx:
            plt.plot(self.lx, self.ly, linestyle='--', linewidth=2.5, label="Local Path")
                
    
    def loadnplot(self, pkg_path):
        
        if PLAN == "Pre":
            #for i in range(1, 6):
            for i in range(1, 4):
                path_x, path_y = self.load_path(f"{pkg_path}/path/pre/woncheon_ajoucity/pp/{i}.txt")    # 시작 path들

                self.parking_paths.append((path_x, path_y))
                self.start_points.append(self.find_closest_idx(self.lx, self.ly, path_x[0], path_y[0]))  # 각 주차 path의 시작점과 가장 가까운 로컬 path 상의 인덱스

        elif PLAN == "Final":
            for i in range(1, 4):
                path_x, path_y = self.load_path(f"{pkg_path}/path/final/woncheon_ajoucity/pp/{i}.txt")  # 시작 path들
                path_x, path_y = path_x[:-self.cut], path_y[:-self.cut]
                
                self.parking_paths.append((path_x, path_y))
                self.start_points.append(self.find_closest_idx(self.lx, self.ly, path_x[0], path_y[0]))  # 각 주차 path의 시작점과 가장 가까운 로컬 path 상의 인덱스

            self.return_x, self.return_y = self.load_path(f"{pkg_path}/path/final/woncheon_ajoucity/pp/return.txt")
                
                
    def handle_parking_decision(self):
        
        print(f"parking_state: {self.parking_state}")
        closest_idx = self.find_closest_idx(self.lx, self.ly)           # 현재 위치와 가장 가까운 local path index
        
        if closest_idx >= self.start_points[self.parking_state] - 3:    # 각 주차 시작점에 도달할 때
            
            if not self.flaged:
                self.flaged_x, self.flaged_y = self.x, self.y
                self.flaged = True
                
            self.stop()
            
            if self.velocity == 0:
                
                if not self.stopped:
                    rospy.logwarn(f"stopped, {self.parking_state + 1}번째 칸")
                    self.cur_picked_path_vis("starting")
                    self.stopped = True
                    
                if PLAN == "Pre":
                    self.obstacles = [
                        (x, y) for (x, y) in self.objects
                        if (LD_X - BET_CONE/2) <= x <= (LD_X + BET_CONE/2)
                        and y >= self.min_y_length - HALF_ROI
                    ]             
                       
                elif PLAN == "Final":
                    self.obstacles = [
                        (x, y) for (x, y) in self.objects
                        if (LD_X - BET_CONE/4) <= x <= (LD_X + BET_CONE/4)
                        and y >= self.min_y_length - HALF_ROI
                    ]             
                
                if self.obstacles:
                    self.obstacle_detected_streak += 1
                    if self.obstacle_detected_streak >= 5:  # 다섯 프레임 이상 지속될 경우
                        self.obstacle_detected_streak = 0
                        self.is_parking_lot_streak = 0
                        
                        self.parking_state += 1
                        if self.parking_state >= len(self.parking_paths):
                            rospy.logwarn("모든 칸이 막혀 있음. 탐색 종료/리셋.")
                            self.is_goal = False
                            self.parking_end = True
                            self.end_parking_publisher.publish(True)
                            
                        rospy.loginfo("막혀있음, 다음 칸으로 이동")
                        self.flaged, self.stopped = False, False
                        
                else:
                    self.is_parking_lot_streak += 1
                    if self.is_parking_lot_streak >= 5:
                        self.obstacle_detected_streak = 0
                        self.is_parking_lot_streak = 0
                        
                        self.is_goal = True
                        self.flaged = False
                        rospy.loginfo("주차 가능, 경로 추종 시작")
                
                print(f"막혀있음: {self.obstacle_detected_streak}, 주차 가능: {self.is_parking_lot_streak}")
            else:
                rospy.logwarn("stopping...")
        else:
            rospy.loginfo_throttle(1.0, "목표 없음, publishing local path...")
            self.parking_path_publisher.publish(self.local_path)
            self.go()
            
            
    def target_on(self):
        
        if not self.idx_calculated:
            
            self.gear_changing_idxs = deque()
            
            if PLAN == "Pre":
                
                px, py = self.parking_paths[self.parking_state]
                
                self.x_list = px + px[-2:self.go_to_back:-1]
                self.y_list = py + py[-2:self.go_to_back:-1]
                self.gear_changing_idxs.extend([len(px), len(self.x_list)])
                
            elif PLAN == "Final":
                
                self.x_list, self.y_list = self.parking_paths[self.parking_state]
                self.seg_lens = [len(self.x_list)]
                self.gear_changing_idxs.append(len(self.x_list))
                
                
                start, end = self.parking_idxs[self.parking_state]
                
                self.x_list += self.return_x[:start]
                self.y_list += self.return_y[:start]
                self.seg_lens.append(len(self.return_x[:start]))
                self.gear_changing_idxs.append(len(self.x_list))
                
                self.x_list += self.return_x[start:end:-1]
                self.y_list += self.return_y[start:end:-1]
                self.seg_lens.append(len(self.return_x[start:end:-1]))
                self.gear_changing_idxs.append(len(self.x_list))
                
                self.x_list += self.return_x[end:start]
                self.y_list += self.return_y[end:start]
                self.gear_changing_idxs.append(len(self.x_list))
                
                
            self.idx_calculated = True
            # 기어가 바뀌는 인덱스 저장 -> parking path가 만들어지고 난 후 처음에 딱 한번만 실행
            
        rospy.loginfo_throttle(1.0, "목표 고정, publishing parking path...")
        self.publish_parking_path()


    def publish_parking_path(self):
        
        if PLAN == "Final":
            print(f"seg_lens: {self.seg_lens}, gear_changing_idxs: {self.gear_changing_idxs}")
        
        self.cur_idx = self.gear_changing_idxs[0]
        closest_idx = self.find_closest_idx(self.x_list[self.prev_idx:self.cur_idx], self.y_list[self.prev_idx:self.cur_idx])
        
        rospy.loginfo_throttle(1.0, f"[gear] {'FWD' if not self.parking_gear else 'REV'} | remaining={len(self.gear_changing_idxs)}")
        
        if self.prev_idx + closest_idx >= self.cur_idx - 2:  # 현재 segment 경로 끝 점에 거의 다다랐을 때
            
            if not self.flaged:
                self.flaged_x, self.flaged_y = self.x, self.y
                self.flaged = True
                
            self.stop()
            
            
            if self.velocity == 0:
                
                self.cur_picked_path_vis("returning")
                self.prev_idx = self.gear_changing_idxs.popleft()
                self.flaged = False
                
                if not self.gear_changing_idxs:
                    self.is_goal = False
                    self.parking_end = True
                    self.end_parking_publisher.publish(True)
                    self.parking_gear = False
                    print("parking end!")
                    return
                
                if len(self.gear_changing_idxs) == (1 if PLAN == "Pre" else 2):
                    rospy.loginfo("waiting for 5 seconds...")
                    rospy.sleep(5)
                    
                if PLAN == "Pre":
                    self.parking_gear = True
                    
                elif PLAN == "Final":
                    if len(self.gear_changing_idxs) % 2: # 남은 gci가 홀수이면.
                        self.parking_gear = True
                    else:
                        self.parking_gear = False
            else:
                rospy.logwarn("stopping...")            
        else:
            self.publish_path(self.x_list, self.y_list, self.prev_idx + 2, self.cur_idx)
            self.go()
            
            
if __name__ == '__main__':
    try:
        Parking_path_gen()
    except rospy.ROSInterruptException:
        pass