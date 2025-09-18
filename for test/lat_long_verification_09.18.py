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

path_name = '08.02.Kcity'
VELOCITY_DATA_FILE_NAME= 'target_vel_data/' + path_name + '.csv'

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


        # ================================== Publisher ================================ $
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size = 1)
        self.pursuit_pub=rospy.Publisher("/pursuit_path", Path, queue_size = 3)
        self.wp_pub = rospy.Publisher('/path_waypoint', Marker, queue_size=5)
        self.erp_msg = erpCmdMsg()
        self.erpStatus_msg  = erpStatusMsg()
        

        
        rospy.on_shutdown(self.close_files)
        self.GAIN_pi_t =4.75
        self.GAIN_x_t =2.25
        self.GAIN_gps =4
        self.BRAKE_GAIN = 0.5

        # =====================================
        self.is_path = False

        self.is_odom = False
        self.is_yaw = False         
        self.is_status = False
        self.is_PathState = False
        self.is_Brake = False
        self.desired_brake = 0
        self.desired_velocity=0

        # rospy.logwarn(f'\n\n\n {self.parking_gear}\n\n\n')
        # mission
        # =====================================
        self.vehicle_yaw=0.
        self.velocity=0.
        self.parking_gear=0
        self.pickup_path_end = False
        self.delivery_path_end = False
        
        # self.delivery_path = Path()
        # self.delivery_path = self.local_path

        # =====================================
        self.current_position=Point()
        self.vehicle_length = 1.04
        self.Path_state="Global_path"
        self.pid_steering = pidControl(p_gain = 0.05, i_gain = 0, d_gain = 0.05, dt = 1/30)
        self.steering_filter = FIRFilter(5)#조향각 이후에 필터 적용
        self.pid= pidControl(p_gain =0.4, i_gain = 0.0, d_gain = 0.001, dt = 0.02)
        self.plot_velocity = Plot_Velocity()

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            is_ready = (self.is_yaw and self.is_path and  self.is_PathState and self.is_odom and self.is_status and self.is_yaw)
            # rospy.logwarn(f'\n\n {self.vehicle_yaw} \n\n')
            if is_ready and self.erpStatus_msg.control_mode == 1:
                self.erp_msg.gear = 0

                steering, target_velocity, brake = self.control_state(self.Path_state)

                if(self.desired_brake == 200):
                    target_velocity = 0
            
                self.erp_msg.steer = steering
                self.erp_msg.speed = target_velocity
                self.erp_msg.brake = int(brake)
                
                print(f"target_velocity: {target_velocity}, current_velocity: {self.velocity}, desired_vel: {self.desired_velocity}, brake: {brake}")
                self.plot_velocity.vel_data(target_velocity, self.velocity, self.desired_velocity, brake)
                self.erp_42_ctrl_pub.publish(self.erp_msg)


                # print("Current_PATH_STATE : {}".format(self.Path_state))
                print("Target_Velocity : {:.2f}, Target_steering : {:.2f}".format(target_velocity/10, math.degrees(steering/2000*0.4922)))
                # print("Current_Velocity : {:.2f}".format(self.velocity/10)) #km/h
                
            else:
                self.init_variable()
                print('Error')
                print(f'is_path:{self.is_path}, is_PathState: {self.is_PathState}, is_odom: {self.is_odom}, is_status: {self.is_status}, is_yaw: {self.is_yaw}')

            rate.sleep()
        
        self.plot_velocity.plot_velocities()
    
    def localPath_callback(self,msg):
        
        self.is_path=True
        self.local_path=msg

    def odom_callback(self,msg):
        
        self.is_odom=True
        
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        # self.vehicle_yaw = msg.pose.pose.position.z

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

    def control_state(self, Path_state):

        brake = 0
        steering =0
        if Path_state == "Global_path":
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

        steering = self.max_value(steering)

        return steering, target_velocity, brake
    

    def max_value(self, steering):

        if steering > 2000:
            steering = 2000
        elif steering < -2000:
            steering = -2000

        return steering
        
    def calc_stanley(self, path):

        # ==================== 전륜 중심 utm 좌표 구하기 ==========================

        if self.Path_state == "Global_path":
            front_utm_x = self.current_position.x + self.GAIN_gps*(self.vehicle_length) * np.cos(self.vehicle_yaw)
            front_utm_y = self.current_position.y + self.GAIN_gps*(self.vehicle_length) * np.sin(self.vehicle_yaw)

        else:
            front_utm_x = self.current_position.x
            front_utm_y = self.current_position.y
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
        #y_2 = - (Way_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (Way_y - self.current_position.y) * np.cos(self.vehicle_yaw)

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

        # self.visualization_heading_WP([front_utm_x, front_utm_y],[Way_x, Way_y], self.current_position,min_index)

        print("===============steering==================")
            
        print("steering: {:.2f}".format(steering))

        print("=================================")
        

        return steering



    
    def control_vel_brake(self): #target_vel과 brake 계산 
        
                        
            goal_velocity = self.desired_velocity
            velocity, brake = self.velocity_pid(goal_velocity)

            return velocity , brake




    def velocity_pid(self, desired_velocity):
        output = self.pid.pid(desired_velocity*10, self.erpStatus_msg.speed)
        if desired_velocity*10 > self.erpStatus_msg.speed-20: #가속하는 경우 목표속도를 입력속도로 집어넣는다.
            velocity = desired_velocity*10
            brake = 1
            print("가속 가속 가속 가속")
        
        else:
            velocity = desired_velocity*10
            brake = -(output*self.BRAKE_GAIN)
            if brake < 1:
                brake =1
            elif brake > 33:
                brake = 33    
            print("감속 감속 감속 감속")

        # if output > 0.0:
        #     if output >200:
        #         output = 200
        #     velocity = output
        #     brake =  1
        # else:
        #     velocity = desired_velocity*10
        #     brake = -(output*self.BRAKE_GAIN)
        #     if brake < 1:
        #         brake =1
        #     elif brake > 33:
        #         brake = 33
        return velocity, brake
    
    
    def visualization_heading_WP(self, Lp, Way, Ego, idx):
        

        pursuit_path=Path()
        pursuit_path.header.frame_id='map'

        read_pose = PoseStamped()
        read_pose.pose.position.x=Ego.x
        read_pose.pose.position.y=Ego.y
        read_pose.pose.position.z=1.
        pursuit_path.poses.append(read_pose)

        read_pose = PoseStamped()
        read_pose.pose.position.x=Lp[0]
        read_pose.pose.position.y=Lp[1]
        read_pose.pose.position.z=1.
        pursuit_path.poses.append(read_pose)
        return
    
    def init_variable(self):

        self.pid_steering.prev_error = 0
        self.pid_steering.i_control  = 0

    def desiredVelocity_callback(self, msg):

        self.is_desiredVel = True
        self.desired_velocity = msg.data



##################  데이터 검증용  ###################

# 1.steering


#   file 닫기
    def close_files(self):
        self.plot_velocity.steering_f.close()


class Plot_Velocity:

    def __init__(self):
        self.vel_target_list = []
        self.vel_current_list = []
        self.vel_desired_list = []
        self.brake_list = []  # <- brake 리스트 추가
        self.time_list = []
        self.counter = 0
        self.steering_f = None
        self.steering_writer = None

    def vel_data(self, TARGET_VELOCITY, CURRENT_VELOCITY, DESIRED_VELOCITY, BRAKE):
        if self.steering_f is None:  # 파일이 열려있지 않으면 처음 한 번만 열기
            self.steering_f = open(VELOCITY_DATA_FILE_NAME, 'w')
            self.steering_writer = csv.writer(self.steering_f)
            self.steering_writer.writerow(["Target Velocity", "Current Velocity", "Desired Velocity", "Brake"])  # 헤더 작성

        TARGET_VELOCITY = TARGET_VELOCITY / 10
        CURRENT_VELOCITY = CURRENT_VELOCITY / 10
        velocity_data = [TARGET_VELOCITY, CURRENT_VELOCITY, DESIRED_VELOCITY, BRAKE]

        # CSV 기록
        self.steering_writer.writerow(velocity_data)
        self.steering_f.flush()

        if not hasattr(self, "start_time"):
            self.start_time = time.time()
        current_time = time.time() - self.start_time
        self.time_list.append(current_time)

        self.vel_target_list.append(TARGET_VELOCITY)
        self.vel_current_list.append(CURRENT_VELOCITY)
        self.vel_desired_list.append(DESIRED_VELOCITY)
        self.brake_list.append(BRAKE)

    def plot_velocities(self):
        plt.figure()
        plt.plot(self.time_list, self.vel_target_list, label="Target Velocity")
        plt.plot(self.time_list, self.vel_current_list, label="Current Velocity")
        plt.plot(self.time_list, self.vel_desired_list, label="Desired Velocity")
        plt.plot(self.time_list, self.brake_list, label="Brake", linestyle='--', color='red')  # brake plot 추가

        plt.xlabel("Time (s)")
        plt.ylabel("Value")
        plt.title("Velocity and Brake over Time")
        plt.legend()
        plt.grid(True)
        plt.show()






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





if __name__ == '__main__':
        
    pure_pursuit()


