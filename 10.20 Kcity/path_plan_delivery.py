#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import math
from std_msgs.msg import Int32, Bool, Float32MultiArray, String, Float64, Float32
from geometry_msgs.msg import PoseStamped , Point
from nav_msgs.msg import Path , Odometry
from math import cos,sin,pi,sqrt,pow,atan2
from scipy.interpolate import CubicSpline     #(25.05.24 cubic spline 개선)
import numpy as np
from visualization_msgs.msg import Marker
import os 
from tracking_msg.msg import TrackingObjectArray


class DeliveryPathPub :
    def __init__(self):
        rospy.init_node('delivery_path_pub', anonymous = True)

        # ----------------------- Subscriber ----------------------- #
        # rospy.Subscriber("/dest_sign_local_coord", Float32MultiArray, self.sign_utm_callback)
        rospy.Subscriber("/local_path", Path, self.localPath_callback)
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        rospy.Subscriber('/path_state', String, self.pathState_callback)
        rospy.Subscriber('/delivery_end_trigger', Bool, self.delivery_trigger_callback)
        rospy.Subscriber('/delivery_return_trigger', Bool, self.delivery_return_callback)
        rospy.Subscriber('/target_sign', Int32, self.target_sign_callback)
        rospy.Subscriber('/fusion_box/fused_3d_box', TrackingObjectArray, self.lidar_callback)

        # ------------------------ Publisher ------------------------- #
        self.delivery_path_pub = rospy.Publisher('/delivery_path',Path, queue_size = 1)
        self.delivery_end_pub = rospy.Publisher('/delivery_end_trigger', Bool, queue_size=1)
        self.Delivery_brake_pub = rospy.Publisher('/delivery_brake', Int32, queue_size=1)

        # ----------------------------------------------------------- #
        self.is_path = False
        self.is_odom = False
        self.is_yaw = False
        self.is_signutm = False
        self.Path_state= None
        self.delivery_start = False
        self.delivery_trigger = False # When delivery end, it's True
        self.delivery_stop_done = False    


        # ===========
        self.close_threshold = 7.1
        self.target_sign_id = None
        self.x_roi = [0, 15]
        self.y_roi = [-7, 7]
        # ===========
        self.current_position = Point()
        self.current_position.x = 0
        self.current_position.y = 0
        self.local_path = Path()
        self.delivery_detect_started = False
        self.START_DETECT_DIST = 5.0   # ✅ 탐지 시작 거리 (m)
        self.STOP_DIST = 3.0           # ✅ 정지 거리 (m)
        # ----------------------------------------------------------- #
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.Path_state == "delivery_path":
                self.delivery_path_pub.publish(self.delivery_path_msg)
            else:
                self.delivery_path_pub.publish(self.local_path)
    # ------------------ callback ---------------------------- #
    def odom_callback(self,msg):
        self.is_odom=True
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        # self.vehicle_yaw = msg.pose.pose.position.z

    def lidar_callback(self, msg: TrackingObjectArray):
        
        for obj in msg.array:

        # 우리가 원하는 type_id만 필터 (3=A1/B1, 4=A2/B2, 5=A3/B3)
            if not hasattr(obj, "type_id") or obj.type_id  not in (3,4,5):
                continue
            if not hasattr(obj, "point"):
                continue

        # LiDAR 좌표에서 거리 계산
            cx, cy = obj.point.x, obj.point.y
            distance = math.sqrt(cx**2 + cy**2)
        # 1) 탐지 시작
            print(not self.delivery_detect_started, self.target_sign_id, obj.type_id, distance <= self.START_DETECT_DIST)
            print(distance)
            print(self.START_DETECT_DIST)
            if (not self.delivery_detect_started and
                self.target_sign_id is not None and
                self.target_sign_id == obj.type_id and
                distance <= self.START_DETECT_DIST):

                
                self.delivery_detect_started = True
                rospy.loginfo(f"[Fusion] Target sign detected at {distance:.2f}m → Tracking start")

        # 딜리버리 정지 조건
            if (not self.delivery_stop_done and 
                self.target_sign_id is not None and 
                self.target_sign_id == obj.type_id and 
                distance <= self.STOP_DIST):   # ← 라이다 거리 3m 기준
                self.apply_brake()
                rospy.logwarn(f"Sign {obj.type_id} reached (~{distance:.1f}m), stopping...")
                rospy.sleep(3)
                self.remove_brake()
                rospy.logwarn("Stop finished, resuming path")
                self.delivery_stop_done = True
                delivery_end_trigger = True
                self.delivery_end_pub.publish(delivery_end_trigger)

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data

    def localPath_callback(self,msg):        
        self.is_path=True
        self.local_path=msg
    def pathState_callback(self, msg: String):
        # Global_path / delivery_path 전환
        self.Path_state = msg.data
        # 딜리버리 진입/복귀 시 플래그 정리
        if self.Path_state == "Delivery_path":
            # 새로운 딜리버리 미션 들어갈 때 플래그 초기화
            self.delivery_detect_started = False
            self.delivery_stop_done = False
        else:
            # Global_path 복귀 시에도 안전하게 리셋
            self.delivery_detect_started = False
            self.delivery_stop_done = False

    def delivery_trigger_callback(self,msg):
        self.delivery_trigger = msg.data
    
    def target_sign_callback(self, msg):
        self.target_sign_id = msg.data

    def apply_brake(self):
        self.Desired_brake_pub.publish(Int32(data=100))

    def remove_brake(self):
        self.Desired_brake_pub.publish(Int32(data=0))

if __name__ == '__main__':
    try:
        test_track = DeliveryPathPub()
    except rospy.ROSInterruptException:
        pass