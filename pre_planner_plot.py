#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, os
import numpy as np
from math import pi,sqrt
from nav_msgs.msg import Path
from std_msgs.msg import Int32, Float32


class pre_planner :
    def __init__(self):
        #subscribe and publish
        rospy.init_node('pre_planner', anonymous=True)
        rospy.Subscriber("current_waypoint", Int32, self.waypoint_callback)
        rospy.Subscriber("global_path", Path, self.global_path_callback)
        self.target_vel_pub = rospy.Publisher("target_vel",Float32, queue_size=1)
        self.is_global_path = False
        self.is_waypoint = False
        self.is_traffic = False
        self.target_vel = 15.0
        self.current_waypoint = 0
        self.traffic_info = 0 # 0 : green , 1 : others
        self.mode = 0 # 0 : safe , 1 : slow , 2 : stop
        self.vel_planning = velocityPlanning(self.target_vel/3.6, 0.15)
        self.end_of_path = 1230


        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(20) # 20hz

        while not rospy.is_shutdown():
            if self.is_traffic and self.is_waypoint:
                self.target_vel = self.velocity_list[self.current_waypoint] * 3.6
                self.target_vel_pub.publish(self.target_vel)

                os.system('clear')
                print("-------------------------------------")
                print("target vel(km/h)", self.target_vel)
                print(f'Current_waypoint : {self.current_waypoint}')
                print("-------------------------------------")

            else:
                os.system('clear')
                if not self.is_waypoint:
                    print("[2] can't subscribe '/current_waypoint' topic...")

            rate.sleep()

    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True
        
    def waypoint_callback(self,msg):
        self.is_waypoint = True
        self.current_waypoint = msg.data



class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton
        self.SAFETY_GAIN = 0.8

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

            #TODO: (6) 도로의 곡률 계산
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)

            #TODO: (7) 곡률 기반 속도 계획
            v_max = sqrt(r*9.8*self.road_friction*self.SAFETY_GAIN)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        pre_planner=pre_planner()
    except rospy.ROSInterruptException:
        pass