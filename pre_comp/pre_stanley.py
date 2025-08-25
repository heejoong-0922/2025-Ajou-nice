#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, os
import numpy as np
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32, Float32


class stanly_controller :
    def __init__(self):
        #subscribe and publish
        rospy.init_node('stanly_controller', anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("Competition_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("current_waypoint", Int32, self.waypoint_callback)
        rospy.Subscriber("target_vel", Float32, self.target_vel_callback)
        rospy.Subscriber("mode", Int32, self.mode_callback)

        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1
        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False
        self.is_waypoint = False
        self.is_target_vel = False
        self.is_mode = False
        self.mode = 0
        self.current_postion = Point()
        self.current_waypoint = 0
        self.target_vel = 0.0
        self.current_vel = 0.0
        self.is_look_forward_point = False
        self.k = 1.15 #stanley constant
        self.v_t = 0.98 #vel constant to get crosstrack error
        self.prev_steering_angle = 0
        self.pid = pidControl()
        self.BRAKE_GAIN = 0.18 #longitudinal velocity related
        self.SLOW_SPEED = 10.0

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            if self.is_path ==True and self.is_odom==True and self.is_status==True and self.is_waypoint==True and self.is_target_vel==True and self.is_mode==True:
                
                if not self.mode == 0:
                    if self.mode == 1:
                        self.target_vel = self.SLOW_SPEED
                    elif self.mode == 2:
                        self.stop()
                        continue

                vehicle_position=self.current_postion
                self.is_look_forward_point= False
                translation=[vehicle_position.x, vehicle_position.y]
                #translation matrix global to local(vehicle)
                t=np.array([
                        [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                        [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                        [0                    ,0                    ,1            ]])
                det_t=np.array([
                    [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
                    [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
                    [0      ,0      ,1                                               ]])
                temp = np.zeros((2,2)) #for save local point of distant minimum path point
                dis_min = 10000
                j = 0
                #roop all path to get distance minimun path point from vehicle
                for num,i in enumerate(self.path.poses) :
                    path_point=i.pose.position
                    global_path_point=[path_point.x,path_point.y,1]
                    local_path_point=det_t.dot(global_path_point)
                    if local_path_point[0] < 0:
                        continue
                    dis = sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                    if dis <= dis_min :
                        dis_min = dis
                        j = num
                        temp[0][0] = local_path_point[0]
                        temp[0][1] = local_path_point[1]
                        temp_global = global_path_point
                        self.is_look_forward_point = True
                    #path point to get path yaw
                    if num == j + 1 :
                        temp[1][0] = local_path_point[0]
                        temp[1][1] = local_path_point[1]
                if self.is_look_forward_point :
                    #get yaw_term through distance minimum path point
                    heading_error = atan2(temp[1][1] - temp[0][1], temp[1][0] - temp[0][0])
                    #get cross track error through distance minimum path point
                    cte = sin(self.vehicle_yaw)*(temp_global[0] - vehicle_position.x) - cos(self.vehicle_yaw)*(temp_global[1] - vehicle_position.y)
                    crosstrack_error = -atan2(self.k * cte, self.current_vel + self.v_t)

                    # get steering angle with heading_error anf crosstrack_error
                    steering_angle = heading_error + crosstrack_error
                    
                    #clip steering angle between (-40, 40) degree
                    steering_angle = np.clip(steering_angle, -2*pi/9, 2*pi/9)
                    self.ctrl_cmd_msg.steering = steering_angle

                    output = self.pid.pid(self.target_vel,self.status_msg.velocity.x*3.6)

                    if output > 0.0:
                        self.ctrl_cmd_msg.accel = output
                        self.ctrl_cmd_msg.brake = 0.0
                    elif output <=0.0:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = -(output*self.BRAKE_GAIN)
                    

                    os.system('clear')
                    print("-------------------------------------")
                    print(" steering (deg) = ", self.ctrl_cmd_msg.steering * 180 / pi)
                    print("target vel(km/h)", self.target_vel)
                    print(f'Current_waypoint : {self.current_waypoint}')
                    print("-------------------------------------")

                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                else :
                    os.system('clear')
                    print("can't find local_forward_point")
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            else :
                os.system('clear')
                if not self.is_path:
                    print("[1] can't subscribe '/local_path' topic...")
                if not self.is_odom:
                    print("[2] can't subscribe '/odom' topic...")  
                if not self.is_status:
                    print("[3] can't subscribe '/Competition_topic' topic")
                if not self.is_target_vel:
                    print("[4] can't subscribe '/target_vel' topic")
                if not self.is_waypoint:
                    print("[5] can't subscribe '/current_waypoint' topic")
                if not self.is_mode:
                    print("[6] can't subscribe '/mode' topic")
            self.is_path = self.is_odom = self.is_target_vel = False
            rate.sleep()

    def status_callback(self, msg):
        self.is_status = True
        self.current_vel = msg.velocity.x
        self.status_msg=msg
    def path_callback(self,msg):
        self.is_path=True
        self.path=msg
    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y
    def waypoint_callback(self,msg):
        self.is_waypoint = True
        self.current_waypoint = msg.data
    def target_vel_callback(self,msg):
        self.is_target_vel = True
        self.target_vel = msg.data
    def mode_callback(self,msg):
        self.is_mode = True
        self.mode = msg.data
    def stop(self):
        self.ctrl_cmd_msg.accel = 0.0
        self.ctrl_cmd_msg.brake = 1.0
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

class pidControl:
    def __init__(self):
        self.p_gain = 0.4
        self.i_gain = 0.0
        self.d_gain = 0.01
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (5) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
    

if __name__ == '__main__':
    try:
        test_track=stanly_controller()
    except rospy.ROSInterruptException:
        pass