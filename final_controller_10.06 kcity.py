from asyncio import set_event_loop
from re import I
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry,Path
#from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from erp_driver.msg import erpCmdMsg, erpStatusMsg
import math
from std_msgs.msg import Float32, String, Float32MultiArray, Int32, Bool
from geometry_msgs.msg import Point32,PoseStamped,Twist
import time
from visualization_msgs.msg import Marker
import csv
import matplotlib.pyplot as plt
import time

path_name = '10.08.FinalTest'

HEADING_OFFSET_FILE_NAME= 'heading_data/' + path_name + '.csv'
STEERING_OFFSET_FILE_NAME = 'steering_data/' + path_name +'.csv'
POSITION_DATA_FILE_NAME = 'position_data/' + path_name +'.csv'
LATERAL_OFFSET_FILE_NAME = 'lateral_data/' +path_name + '.csv'
SPEED_DATA_FILE_NAME = 'speed_data/' + path_name + '.csv'





class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        # =============================== subscriber =============================== #
        rospy.Subscriber("/local_path", Path, self.localPath_callback)

        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)  
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber('/desired_velocity', Int32, self.desiredVelocity_callback)
        rospy.Subscriber('/path_state', String, self.pathState_callback)
        rospy.Subscriber('/obs_small_steer', Int32, self.obsSmall_callback)
        rospy.Subscriber('/obs_big_steer', Int32, self.obsBig_callback)
        rospy.Subscriber('/parking_path', Path, self.parkingPath_callback)
        rospy.Subscriber('/parking_velocity', Int32, self.parkingVelocity_callback)
        rospy.Subscriber('/parking_gear', Int32, self.parkingGear_callback)

        # ================================== Publisher ================================ $
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size = 1)
        self.pursuit_pub=rospy.Publisher("/pursuit_path", Path, queue_size = 3)
        self.wp_pub = rospy.Publisher('/path_waypoint', Marker, queue_size=5)
        self.erp_msg = erpCmdMsg()
        self.erpStatus_msg  = erpStatusMsg()
        
        self.position_f = open(POSITION_DATA_FILE_NAME, 'w')
        self.steering_f = open(STEERING_OFFSET_FILE_NAME, 'w')
        self.heading_f = open(HEADING_OFFSET_FILE_NAME, 'w')
        self.lateral_f = open(LATERAL_OFFSET_FILE_NAME, 'w')
        self.speed_f = open(SPEED_DATA_FILE_NAME, 'w')

        self.steering_writer = csv.writer(self.steering_f)
        self.position_writer = csv.writer(self.position_f)
        self.heading_writer = csv.writer(self.heading_f)
        self.lateral_writer = csv.writer(self.lateral_f)
        self.speed_writer = csv.writer(self.speed_f)


        rospy.on_shutdown(self.close_files)
        #stanley parameter
        #======================================
        self.GAIN_pi_t =4.75
        self.GAIN_x_t =2.25
        self.GAIN_gps =4
        self.BRAKE_GAIN = 0.06
        
        #initial parameter
        # =====================================
        self.is_path = False
        self.is_odom = False
        self.is_yaw = False         
        self.is_status = False
        self.is_PathState = False
        self.is_parkingVel = False        
        self.is_gear = False
        self.is_parking_path = False
        self.is_big_steer = False
        self.is_small_steer = False
        self.desired_velocity=0
        self.vehicle_yaw=0.
        self.velocity=0.
        self.parking_gear=0

        self.current_position=Point()
        self.vehicle_length = 1.04
        self.Path_state="Global_path"
        self.steering_filter = FIRFilter(5)#조향각 이후에 필터 적용
        self.pid= pidControl(p_gain =5, i_gain = 0.0, d_gain = 0.001, dt = 0.02)


        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            is_ready = (self.is_path and  self.is_PathState and self.is_odom and self.is_status and self.is_yaw)
            if is_ready and self.erpStatus_msg.control_mode == 1:
                self.erp_msg.gear = 0

                steering, target_velocity, brake, gear = self.control_state(self.Path_state)

                self.erp_msg.steer = steering
                self.erp_msg.speed = target_velocity
                self.erp_msg.brake = brake
                self.erp_msg.gear = gear
                
                self.record_data(steering)

                self.erp_42_ctrl_pub.publish(self.erp_msg)

                print("Target_Velocity : {:.2f}, Target_steering : {:.2f}".format(target_velocity/10, math.degrees(steering/2000*0.4922)))
                
            else:
                print('Error')
                print(f'is_path:{self.is_path}, is_PathState: {self.is_PathState}, is_odom: {self.is_odom}, is_status: {self.is_status}, is_yaw: {self.is_yaw}')

            rate.sleep()
        


    def control_state(self, Path_state):
        target_velocity =0
        brake = 0
        steering =0
        gear = 0

        if Path_state == "Global_path":
            self.path = self.local_path
            steering = self.calc_stanley(self.path)
            target_velocity, brake =self.control_vel_brake()

        elif Path_state == "Pickup":
            self.path = self.local_path
            steering = self.calc_stanley(self.path)
            target_velocity, brake =self.control_vel_brake() 

        elif Path_state == "Intersection":
            self.path = self.local_path
            steering = self.calc_stanley(self.path)
            target_velocity, brake =self.control_vel_brake() 
        
        elif Path_state == "Curve":

            self.path = self.local_path
            steering = self.calc_stanley(self.path)
            target_velocity, brake =self.control_vel_brake() 

        elif Path_state == "Small_Obstacle_avoiding_path":

            steering = self.obs_small_steer if self.is_small_steer else 0
            target_velocity, brake = self.control_vel_brake()

        elif Path_state == "Big_Obstacle_avoiding_path":

            steering = self.obs_big_steer if self.is_big_steer else 0
            target_velocity, brake = self.control_vel_brake()

        elif Path_state == "Parking_path":

            self.path = self.parking_path if self.is_parking_path else self.local_path
            steering = self.calc_stanley(self.path)
            target_velocity, brake =self.control_vel_brake() 
            gear = self.parking_gear

        elif Path_state == "Delivery_path":

            self.path = self.local_path
            steering = self.calc_stanley(self.path)
            target_velocity, brake =self.control_vel_brake() 

        else:
            self.path = self.local_path
            steering = self.calc_stanley(self.path)
            target_velocity, brake =self.control_vel_brake()


        steering = self.max_value(int(steering))
        target_velocity = self.max_speed(int(target_velocity))
        brake = self.max_speed(int(brake))
    
        return steering, target_velocity, brake, gear
    
    def max_speed(self, velocity):

        if velocity > 200:
            velocity = 200
        elif velocity < -200:
            velocity = -200

        return velocity
            

    def max_value(self, steering):

        if steering > 2000:
            steering = 2000
        elif steering < -2000:
            steering = -2000

        return steering
        
    def calc_stanley(self, path):

        # ==================== 전륜 중심 utm 좌표 구하기 ==========================
        front_utm_x = self.current_position.x + self.GAIN_gps*(self.vehicle_length) * np.cos(self.vehicle_yaw)
        front_utm_y = self.current_position.y + self.GAIN_gps*(self.vehicle_length) * np.sin(self.vehicle_yaw)

        # ==================== 현재 속도 구하기 ===================================

        curr_velocity = self.erpStatus_msg.speed
        curr_velocity_kmh = curr_velocity / 10

        # ==================== local_path 불러오기 ================================

        ref_x = []  
        ref_y = []  

        for pose in path.poses:
            ref_x.append(pose.pose.position.x)
            ref_y.append(pose.pose.position.y)

        # ==================== x(t) 구하기 ========================================

        dis_P2 = np.sqrt((np.array(ref_x) - front_utm_x)**2 + (np.array(ref_y) - front_utm_y)**2)
        
        # 최솟값 인덱스 찾기
        min_index = np.argmin(dis_P2)

        # 해당 인덱스에 대한 경로 위치 좌표
        Way_x = ref_x[min_index]
        Way_y = ref_y[min_index]

        # 현재 위치와 경로 위치의 차이 계산
        x_2 = (Way_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (Way_y - self.current_position.y) * np.sin(self.vehicle_yaw)
        y_2 = - (Way_x - front_utm_x) * np.sin(self.vehicle_yaw) + (Way_y - front_utm_y) * np.cos(self.vehicle_yaw)        

        #####vector 외적을 이용한 횡방향 오차 계산####
        # heading_vec =np.array([np.cos(self.vehicle_yaw),np.sin(self.vehicle_yaw)])
        # error_vec = np.array([Way_x-front_utm_x, Way_y-front_utm_y])
        # cross_track_error =np.cross(heading_vec,error_vec)
        # self.x_t = -cross_track_error
        
        #####local 좌표를 이용한 횡방향 오차 계산#####
        self.x_t = -y_2 

        #####original x_t ###########
        # if y_2 > 0: #차량 기준 경로점 왼쪽
        #     self.x_t = -np.min(dis_P2) 
        # else:
        #     self.x_t = np.min(dis_P2) 

        # ====================== pi(t) 구하기 =====================================

        # min_index + 1을 사용하고, 범위를 벗어나는 경우 예외 처리
        if min_index + 1 < len(ref_x):
            delta_east = ref_x[min_index + 1] - ref_x[min_index]
            delta_north = ref_y[min_index + 1] - ref_y[min_index]
        else:  # min_index가 ref_x의 마지막 인덱스일 때
            if min_index - 1 >= 0:
                delta_east = ref_x[min_index] - ref_x[min_index - 1]
                delta_north = ref_y[min_index] - ref_y[min_index - 1]
            else: # 경로 데이터가 1개일때
                delta_east = 0  # min_index가 0일 때 (예외 처리)
                delta_north = 0

        path_yaw = math.atan2(delta_north, delta_east)

        pi_t = self.vehicle_yaw - path_yaw

        pi_t = (pi_t + np.pi) % (2 * np.pi) - np.pi

        # ======================= 조향각 구하기 ======================================

        if curr_velocity_kmh != 0:
                steering = self.GAIN_pi_t*pi_t + np.arctan(self.GAIN_x_t * self.x_t / curr_velocity_kmh) 

        else: # 속도가 0일때 예외처리 
            steering = pi_t

        rospy.logwarn(f'\n x_t : {self.x_t} \n pi_t : {pi_t} \n path_yaw : {path_yaw}')
        steering_raw = 2000 * (steering * 0.422)                    #조향각 이후에 필터 적용
        steering_filtered = self.steering_filter.apply(steering_raw)#조향각 이후에 평균냄
        steering = int(steering_filtered)                           #실수형 정수형 변환

        print("===============steering==================")
            
        print("steering: {:.2f}".format(steering))

        print("=================================")
    
        return steering



    
    def control_vel_brake(self): #target_vel과 brake 계산 
        
            if self.is_parkingVel: #주차 시에는 parking_velocity를 이용해서 목표속도 설정
                goal_velocity = self.parking_velocity
                velocity, brake = self.velocity_pid(goal_velocity)
                
                if goal_velocity == 0:
                    brake = 100
            else:
                goal_velocity = self.desired_velocity
                velocity, brake = self.velocity_pid(goal_velocity)


            return velocity , brake




    def velocity_pid(self, desired_velocity):
        
        output = self.pid.pid(desired_velocity*10, self.erpStatus_msg.speed)

        if desired_velocity*10 -self.erpStatus_msg.speed > 20: #현재속도보다 목표속도가 2km/h 더 클때
            velocity = 200
            brake = 1
            print("가속 가속 가속 가속")
        
        elif abs(desired_velocity*10-self.erpStatus_msg.speed) <= 20: #속도 유지하는 경우 목표속도를 입력속도로 집어넣는다.
            
            velocity = desired_velocity*10
            brake = 1
            print("속도유지 속도유지 속도유지")
        
        else:
            if self.Path_state == "Pickup_path":
                self.BRAKE_GAIN = 0.1
            else:
                self.BRAKE_GAIN = 0.06    
            velocity = desired_velocity*10
            brake = -(output*self.BRAKE_GAIN)
            if brake < 1:
                brake =1   
            print("감속 감속 감속 감속")

        return velocity, brake
#===================== callback 함수 =========================
    def desiredVelocity_callback(self, msg):

        self.is_desiredVel = True
        self.desired_velocity = msg.data
    
    def parkingVelocity_callback(self,msg):
        self.is_parkingVel = True        
        self.parking_velocity = msg.data

    def parkingGear_callback(self,msg):
        self.is_gear = True
        self.parking_gear = msg.data

    def obsSmall_callback(self, msg):
        self.is_small_steer = True
        self.obs_small_steer = msg.data

    def obsBig_callback(self, msg):
        self.is_big_steer = True
        self.obs_big_steer = msg.data

    def localPath_callback(self,msg):
        self.is_path=True
        self.local_path=msg

    def parkingPath_callback(self,msg):
        self.is_parking_path = True
        self.parking_path = msg

    def odom_callback(self,msg):
        self.is_odom=True
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        if self.parking_gear == 2:
            self.vehicle_yaw = msg.data + np.pi
        else:
            self.vehicle_yaw = msg.data 

        if self.vehicle_yaw > np.pi:
            self.vehicle_yaw -= 2*np.pi
        elif self.vehicle_yaw < -np.pi:
            self.vehicle_yaw += 2*np.pi

    def pathState_callback(self, msg):
        self.is_PathState=True
        self.Path_state = msg.data

    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg
        self.velocity = self.erpStatus_msg.speed
        self.brake = self.erpStatus_msg.brake

##################  데이터 검증용  ###################

# 1.steering
    def steering_data(self, DESIRED_STEER):
        
        DESIRED_STEER_DEG = DESIRED_STEER/2000*(28.2) # 스탠리
        current_steer_deg = self.erpStatus_msg.steer/2000 * (28.2)
        steering_data = [DESIRED_STEER_DEG, current_steer_deg]
        self.steering_writer.writerow(steering_data)
        self.steering_f.flush()

#2. erp 실제 주행 경로 gps 데이터
    def position_data(self): 
        position_data = [self.current_position.x, self.current_position.y]
        self.position_writer.writerow(position_data)
        self.position_f.flush()

#3. heading 데이터
    def heading_data(self):
        heading_data = [self.vehicle_yaw]
        self.heading_writer.writerow(heading_data)
        self.heading_f.flush()

#4. lateral 데이터
    def lateral_data(self):
        lateral_data = [self.x_t]
        self.lateral_writer.writerow(lateral_data)
        self.lateral_f.flush()

#5. 속도 데이터
    def speed_data(self, target_velocity, current_velocity, desired_velocity, brake):
        # ERP42 속도는 0.1 km/h 단위 → /10 해서 km/h
        target_kmh = target_velocity / 10
        current_kmh = current_velocity / 10
        desired_kmh = desired_velocity
        speed_data = [target_kmh, current_kmh, desired_kmh, brake]
        self.speed_writer.writerow(speed_data)
        self.speed_f.flush()

#   file 닫기
    def close_files(self):
        self.position_f.close()
        self.steering_f.close()
        self.heading_f.close()
        self.lateral_f.close()
        self.speed_f.close()

    def record_data(self, steering):

        if self.Path_state == "Global_path" or self.Path_state == "Pickup" or self.Path_state == "Curve" or self.Path_state == "Intersection":
            self.steering_data(steering)
            self.position_data()
            self.heading_data()
            self.lateral_data()
            self.speed_data(self.erp_msg.speed, self.velocity, self.desired_velocity, self.brake/10)

        else:
            pass



class pidControl:
    def __init__(self, p_gain, i_gain , d_gain, dt):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = dt
    
    def pid(self,target_vel, current_vel): #(desired_velocity*10, self.erpStatus_msg.speed)
        error = target_vel - current_vel

        #TODO: (4) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
class FIRFilter:
    def __init__(self, window_size_straight=5, window_size_curve=3, steering_threshold=1.0): #1이 직선과 곡선의 기준 1을 넘으면 곡선으로 간주-> window 사이즈 줄임
        self.window_size_straight = window_size_straight                                     #조정이 필요 직선과 곡선의 기준이 모호
        self.window_size_curve = window_size_curve                                           #직선은 5정도면 기존과 동일, 곡선은 변화필요
        self.steering_threshold = steering_threshold
        self.window_size = window_size_straight
        self.buffer = []
        self.prev_value = None

    def apply(self, new_value):
        # 조향 변화량 기반 동적 window_size 조정
        if self.prev_value is not None:
            steering_change = abs(new_value - self.prev_value)
            if steering_change > self.steering_threshold:
                self.window_size = self.window_size_curve  # 커브
            else:
                self.window_size = self.window_size_straight  # 직선
        self.prev_value = new_value

        # FIR 필터 적용
        self.buffer.append(new_value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer)


if __name__ == '__main__':
        
    pure_pursuit()


