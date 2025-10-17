
import rospy
import rospkg
from std_msgs.msg import Int32, Bool, Float32MultiArray, String, Float32
from microstrain_inertial_msgs.msg import FilterHeading
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from erp_driver.msg import erpCmdMsg, erpStatusMsg
from tracking_msg.msg import TrackingObjectArray
import time, math

class State_machine:
    def __init__(self):
        rospy.init_node('Behavior_decision', anonymous=True)

        #--------------------------------Subscriber------------------------------------
        rospy.Subscriber('/current_waypoint', Int32, self.index_callback) # ERP42가 경로상 몇 번째 way_point에 위치한지 받아오기
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback) # ERP42 위치 정보 받기 
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        rospy.Subscriber('/global_path',Path, self.global_path_callback)
        rospy.Subscriber("/traffic_light", Int32, self.traffic_light_callback)
        rospy.Subscriber("/delivery_end_trigger", Bool, self.dleivery_end_trigger_callback)
        rospy.Subscriber('/Y2L_fusion/fused_3d_box', TrackingObjectArray, self.object_callback, queue_size=1)


        #--------------------------------Publisher--------------------------------------
        self.Desired_velocity_pub = rospy.Publisher('/desired_velocity', Int32, queue_size=1) # 원하는 속도를 제어기에 넘기기
        self.Desired_brake_pub = rospy.Publisher('/desired_brake', Int32, queue_size=1)
        self.Path_pub = rospy.Publisher('/path_state', String, queue_size= 1) # 전역 경로로 주행하게 하기
        self.State_pub = rospy.Publisher('/State', String, queue_size= 1)
        self.Traget_sign_pub = rospy.Publisher('/target_sign',Int32, queue_size = 1)
        #==================================Initial_Parameter =================================
        '''
        신호등: /traffic_light
        stop : 5
        red_left_arrow : 6
        green_left_arrow : 7
        go : 8
        '''
        self.State = "Unready"
        self.Path_state="Global_path"
        self.traffic_light = 0
        self.index = 0
        self.sign_type_id = None #표지판의 초기값.
        self.is_index = False
        self.is_yaw = False
        self.is_odom=False
        self.pick_end = False
        self.is_status = False
        self.is_path = False
        self. delivery_end_trigger = False

        #-----------------intersection ------------------
        self.curve_6_finish = False
        self.Intersection_7_finish = False

        #===================================Index_Parameter======================================

        self.pickup_start_index = 5
        self.pickup_stop_index = 77 #픽업이 정지해야 하는 구간
        self.pickup_end_index = 90
        
        self.traffic_2_stop = 151 #traffic 2 정지선
        self.traffic_2_end = 351 #traffic 2가 안보이기 시작하는 지점

        self.traffic_3_stop = -1 #traffic 3 정지선
        self.traffic_3_end = 400 #traffic 3이 안보이기 시작하는 지점
        
        self.big_obs_start_index = 550
        self.big_obs_end_index = 670

        self.traffic_5_stop = -1 #traffic 5 정지선
        self.traffic_5_end = 680 #traffic 5가 안보이기 시작하는 지점

        self.curve_6_start = 690 #곡선구간 6번이 시작되는 지점
        self.curve_6_stop = -1  #곡선 구간 6본 정지
        self.curve_6_end = 695   #곡선구간 6번이 끝나는 지점

        self.traffic_7_stop = -1 #7번 횡단보도 앞 정지
        self.curve_7_start = 700  #곡선구간 7번이 시작되는 지점
        self.curve_7_end = 705    #곡선 구간 7번이 멈추는 지점

        self.traffic_8_stop = -1 #traffic 8 정지선
        self.traffic_8_end = 710 #traffic 8이 끝나는 지점

        self.delivery_start_index = 715  #배달 시작
        self.delivery_end_index = 720    #베달 끝

        self.traffic_10_stop = -1 #traffic 10 정지선
        self.traffic_10_end = 725  #traffic 10이 안보이기 시작하는 지점

        self.traffic_11_stop = -1 #traffic 11 정지선
        self.traffic_11_end = 730  #traffic 11이 안보이기 시작하는 지점

        self.small_obs_start_index = 773 #소형 장애물 시작
        self.small_obs_end_index = 969 # 842   #소형 장애물 끝

        self.parking_start_index = 970    #주차 시작
        self.parking_end_index = 1050      #주차 끝

        #=========================== Main ==================================
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.index_State()
            print(f"State: {self.State}")
            rate.sleep()

        #===================================================================



    def index_State(self):
        
        if not self.is_index and not self.is_odom and not self.is_yaw and not self.is_status and not self.is_path:
            print("Before Start, Sensor input check")
        #================ 시작점부터 픽업 전까지 ==============================
        
        elif 0 <= self.index < self.pickup_start_index:
            
            print("구간: 시작점부터 픽업 전까지")
            
            self.Path_state= "Global_path"
            self.Path_pub.publish(self.Path_state)
            self.vel_17()

        #======================== 픽업 ==================================

        elif self.pickup_start_index <= self.index < self.pickup_end_index: 

            print("구간: 픽업 구간 ")
            
            self.Path_state= "Pickup"
            self.Path_pub.publish(self.Path_state)

            if self.pickup_stop_index -17 <= self.index  and not self.pick_end:
                self.stop()
                self.apply_brake()
                time.sleep(3)
                self.remove_brake()
                self.pick_end = True

            else:
                self.vel_7()

        #======================== 픽업 후부터 traffic 2번까지 ================================

        elif self.pickup_end_index <= self.index < self.traffic_2_end:
            
            print("구간: 픽업 후부터 2번 traffic 까지 ")

            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)
            
            if self.traffic_light == 8:
                rospy.logwarn('green light 인식됨')
                self.vel_20()

            else:
                rospy.logwarn('red light or yellow light 인식됨')
                if self.traffic_2_stop-53 <= self.index <self.traffic_2_stop+ 3:
                    self.stop()
                else:
                    self.vel_20()

        
        #===================== traffic 2번부터 traffic 3번까지 ==================================

        elif self.traffic_2_end <= self.index < self.traffic_3_end:
            
            print("구간: traffic 2번부터 traffic 3번까지")

            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)
            
            if self.traffic_light == 8:
                rospy.logwarn('green light 인식됨')
                self.vel_20()

            else:
                rospy.logwarn('red light 인식됨')
                if self.traffic_3_stop-53 <= self.index < self.traffic_3_stop:
                    self.stop()
                else:
                    self.vel_20()

        #==================== traffic 3번 부터 대형 장애물 전까지 ===================================
        
        elif self.traffic_3_end <= self.index < self.big_obs_start_index:
        
            print("구간: traffic 3번 부터 대형 장애물 전까지")

            self.Path_state = "Global_path"
            self.Path_pub.publish(self.Path_state)
            self.vel_12()

        #================================ 대형 장애물 ===================================

        elif self.big_obs_start_index <= self.index < self.big_obs_end_index:
        
            print("구간: 대형 장애물 구간")

            self.Path_state="Big_Obstacle_avoiding_path"
            self.Path_pub.publish(self.Path_state)
            self.big_obs_avoid_vel()

        #======================대형 장애물 후부터 traffic 5번까지 ==========================
        
        elif self.big_obs_end_index <= self.index < self.traffic_5_end:
        
            print("구간: 대형 장애물 후부터 traffic 5번까지")

            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)

            if self.traffic_light == 8:
                rospy.logwarn('green light 인식됨')
                self.vel_20()

            else:
                rospy.logwarn('red light 인식됨')
                if self.traffic_5_stop-53 <= self.index < self.traffic_5_stop:
                    self.stop()
                else:
                    self.vel_20()
            
        #==================== traffic 5번 부터 6번 곡선구간 전까지 ===================================
        
        elif self.traffic_5_end <= self.index < self.curve_6_start- 50: 
        
            print("구간: traffic 5번 부터 6번 곡선 구간 전까지")

            self.Path_state = "Global_path"
            self.Path_pub.publish(self.Path_state)
            self.vel_20()

        #==================== 6번 곡선구간===================================
        
        elif self.curve_6_start-50 <= self.index < self.curve_6_end:   #커브들어가기 50개 index전부터 속도 5로 감속하기 
        
            print("구간: 6번 곡선 구간")
            
            self.Path_state = "Curve"
            self.Path_pub.publish(self.Path_state)
            if self.curve_6_start -50 <= self.index < self.curve_6_stop-20:
                rospy.logwarn('before curve 감속')
                self.vel_5()

            elif self.curve_6_stop-20 <= self.index < self.curve_6_stop and self.curve_6_finish:
                rospy.logwarn('stop') 
                self.stop()
                self.apply_brake()
                time.sleep(3)
                self.remove_brake()
                self.curve_6_finish = True

            else:
                self.vel_12()
                rospy.logwarn('curve 6 index case , but not stop point')

        #===================6번 곡선구간 후부터 7번 곡선구간 전 =======================

        elif self.curve_6_end <= self.index < self.curve_7_start:
        
            print("구간: 6-7번 사이 직선구간")

            self.Path_state = "Intersection"
            self.Path_pub.publish(self.Path_state)

            if self.traffic_7_stop -53 <= self.index < self.traffic_7_stop and not self.Intersection_7_finish:
                rospy.logwarn('first stop not end')
                self.stop()
                time.sleep(3)
                self.Intersection_7_finish = True

            else:
                self.vel_20()

        #==================== 7번 곡선구간 ===================================

        elif self.curve_7_start <= self.index < self.curve_7_end:

            print("구간: 7번 곡선 구간")

            self.Path_state = "Curve"
            self.Path_pub.publish(self.Path_state)

            self.vel_12()

        #==================== 7번-8번 직선 구간 ==============================

        elif self.curve_7_end <= self.index < self.traffic_8_end:

            print("구간: 7-8번 직선 구간")
            
            self.Path_state="Intersection"
            self.Path_pub.publish(self.Path_state)

            if self.traffic_light == 8:
                rospy.logwarn('green light 인식됨')
                self.vel_20()

            else:
                rospy.logwarn('red light 인식됨')
                if self.traffic_8_stop-53 <= self.index < self.traffic_8_stop:
                    self.stop()
                else:
                    self.vel_20()
            
        #==================== 8번-배달 시작 전까지==============================

        elif self.traffic_8_end <= self.index < self.delivery_start_index:

            print("구간: 8번 끝부터 배달 시작 전까지")
            
            self.Path_state = "Global_path"
            self.Path_pub.publish(self.Path_state)

            self.vel_15()

        
        # ======================= 배달 구간 ===================================

        elif self.delivery_start_index <= self.index < self.delivery_end_index:

            print("구간: 배달 구간")
            
            self.Path_state = "Delivery_path"
            self.Path_pub.publish(self.Path_state)
            self.sign_pub() #픽업 시 받은 표지판 id pub

            if self.delivery_end_trigger: #배달이 끝난 후
                self.vel_15()
            
            else:                         #배달 진행 중
                self.vel_7()

        # ======================= 배달 후 부터 10번 traffic 끝까지 ==================

        elif self.delivery_end_index <= self.index < self.traffic_10_end:
        
            print("구간: 배달 후 부터 10번 traffic 끝까지")

            self.Path_state = "Intersection"
            self.Path_pub.publish(self.Path_state)

            if self.traffic_light == 8:
                rospy.logwarn('green light 인식됨')
                self.vel_15()

            else:
                rospy.logwarn('red light 인식됨')
                if self.traffic_10_stop-37 <= self.index < self.traffic_10_stop:
                    self.stop()
                else:
                    self.vel_15()

        # ======================= 10번 traffic 끝부터 11번 traffic 끝까지==================

        elif self.traffic_10_end <= self.index < self.traffic_11_end:
        
            print("구간: 10번 traffic 끝부터 11번 traffic 끝까지")

            self.Path_state = "Intersection"
            self.Path_pub.publish(self.Path_state)

            if self.traffic_light == 8:
                rospy.logwarn('green light 인식됨')
                self.vel_15()

            else:
                rospy.logwarn('red light 인식됨')
                if self.traffic_11_stop-37 < self.index < self.traffic_11_stop:
                    self.stop()
                else:
                    self.vel_15()

        # ======================= 11번 traffic 끝부터 소형 장애물 회피 시작 전까지==================

        elif self.traffic_11_end <= self.index < self.small_obs_start_index:
        
            print("구간: 11번 traffic 끝부터 소형 장애물 회피 시작 전까지")

            self.Path_state = "Global_path"
            self.Path_pub.publish(self.Path_state)

            self.vel_12()

        # ======================= 소형 장애물 회피 구간  ========================================

        elif self.small_obs_start_index <= self.index < self.small_obs_end_index:

            print("구간: 소형 장애물 회피 구간")


            self.Path_state = "Small_Obstacle_avoiding_path"
            self.Path_pub.publish(self.Path_state)

            self.small_obs_avoid_vel()

        # ======================= 소형 장애물 회피 후부터 주차 전까지 ================================

        elif self.small_obs_end_index <= self.index < self.parking_start_index:

            print("구간: 소형 장애물 회피 후부터 주차 전까지")

            self.Path_state = "Global_path"
            self.Path_pub.publish(self.Path_state)

            self.vel_12()

        # ===================================== 주차 구간 =========================================

        elif self.parking_start_index <= self.index < self.parking_end_index:

            print("구간: 주차 구간")

            self.Path_state = "Parking_path"
            self.Path_pub.publish(self.Path_state)

            self.parking_vel()

        #======================= 주차 끝나고 도착지점까지 ==========================================

        else:
            print("구간: 도착 지점까지")

            self.Path_state = "Global_path"
            self.Path_pub.publish(self.Path_state)

            self.vel_20()
































#------------------------------Callback_function--------------------------------
    def index_callback(self, msg):
        self.is_index = True
        self.index = msg.data
    
    def odom_callback(self,msg):
        self.is_odom=True
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data

    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg
        self.velocity = self.erpStatus_msg.speed
        
    def traffic_light_callback(self, msg):
        self.traffic_light = msg.data

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg

    def dleivery_end_trigger_callback(self, msg):
        self.delivery_end_trigger = msg

    def object_callback(self, msg : TrackingObjectArray):
        if self.Path_state == "Pickup":
            for obj in msg.array:
                if not hasattr(obj, "bbox") or not hasattr(obj.bbox, "points") or len(obj.bbox.points) < 8: continue
                sign = getattr(obj, "type_id", None)
                if sign not in (3, 4, 5): continue
                self.sign_type_id = obj.type_id

    def sign_pub(self):
        msg = Int32()
        msg.data = self.sign_type_id
        self.Traget_sign_pub.publish(msg) #인식한 표지판의 id



#=================== 속도 함수 ====================================
    def vel_20(self): 
        int_msg = Int32()
        int_msg.data = 20
        self.Desired_velocity_pub.publish(int_msg) 
    
    def vel_17(self): 
        int_msg = Int32()
        int_msg.data = 15
        self.Desired_velocity_pub.publish(int_msg) 
    
    def vel_15(self): 
        int_msg = Int32()
        int_msg.data = 15
        self.Desired_velocity_pub.publish(int_msg) 

    def vel_12(self): 
        int_msg = Int32()
        int_msg.data = 12
        self.Desired_velocity_pub.publish(int_msg) 

    def vel_9(self):
        int_msg = Int32()
        int_msg.data = 9
        self.Desired_velocity_pub.publish(int_msg)

    def vel_7(self): 
        int_msg = Int32()
        int_msg.data = 7
        self.Desired_velocity_pub.publish(int_msg) 
    
    def vel_5(self): 
        int_msg = Int32()
        int_msg.data = 5
        self.Desired_velocity_pub.publish(int_msg) 

    def big_obs_avoid_vel(self):
        int_msg = Int32()
        int_msg.data = 6
        self.Desired_velocity_pub.publish(int_msg) 

    def small_obs_avoid_vel(self):
        int_msg = Int32()
        int_msg.data = 2
        self.Desired_velocity_pub.publish(int_msg) 

    def parking_vel(self):
        int_msg = Int32()
        int_msg.data = 3
        self.Desired_velocity_pub.publish(int_msg) 

    def stop(self):
        int_msg = Int32()
        int_msg.data = 0
        self.Desired_velocity_pub.publish(int_msg)

    def apply_brake(self):
        int_msg = Int32()
        int_msg.data = 200
        self.Desired_brake_pub.publish(int_msg)

    def remove_brake(self):
        int_msg = Int32()
        int_msg.data = 0
        self.Desired_brake_pub.publish(int_msg)

if __name__ == '__main__':
    try:
        State_machine()

    except rospy.ROSInterruptException:
        pass
