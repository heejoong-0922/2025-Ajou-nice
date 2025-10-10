import matplotlib.pyplot as plt
import numpy as np
import rospkg
import rospy
from scipy.ndimage import gaussian_filter1d

# File Name (absolute path)
# base_path = '/home/ajou/control_ws/src/gpsimu/scripts/2025/lateral verification/'
path_name = '06.30.woncheon1'
HEADING_OFFSET_FILE_NAME = 'heading_data/' + path_name + '.csv'
STEERING_OFFSET_FILE_NAME =  'steering_data/' + path_name + '.csv'
POSITION_DATA_FILE_NAME =  'position_data/' + path_name + '.csv'
LATERAL_OFFSET_FILE_NAME = 'lateral_data/' + path_name + '.csv'
SPEED_DATA_FILE_NAME = 'speed_data/' + path_name + '.csv'

# frameRate
frameRate = 30


heading_offset_data = []
steering_pid_data = []
steering_current_data = []
steering_stanely_data =[]
position_data_x = []
position_data_y = []
global_x_data = []
global_y_data = []
lateral_offset_data =[]
speed_target_data = []
speed_current_data = []
speed_desired_data = []


def read_file():
    with open(HEADING_OFFSET_FILE_NAME, 'r') as heading_f:
        for line in heading_f:
            heading_offset_data.append(float(line.strip()))
    
    with open(STEERING_OFFSET_FILE_NAME, 'r') as steering_f:
        for line in steering_f:
            x, y  = line.strip().split(',')
            steering_stanely_data.append(float(x))
            steering_current_data.append(float(y))

    with open(POSITION_DATA_FILE_NAME, 'r') as position_f:
        for line in position_f:
            x, y = line.strip().split(',')
            position_data_x.append(float(x))
            position_data_y.append(float(y))
    
    with open(LATERAL_OFFSET_FILE_NAME, 'r') as lateral_f:
        for line in lateral_f:
            lateral_offset_data.append(float(line.strip()))

    with open(SPEED_DATA_FILE_NAME, 'r') as speed_f:
        for line in speed_f:
            t, c, d = line.strip().split(',')
            speed_target_data.append(float(t))
            speed_current_data.append(float(c))
            speed_desired_data.append(float(d))


    #==================== globalpath data 받아오기==============
    pkg_name = 'gpsimu'
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(pkg_name)
    global_path_name = '06.28.woncheon1'
    file_path = pkg_path + '/path' + '/' + global_path_name + '.txt'

    with open(file_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            x, y, _ = line.strip().split('\t')
            global_x_data.append(float(x))
            global_y_data.append(float(y))

    #====================

def compute_normals(x, y):
    dx = np.diff(x, append=x[-1])
    dy = np.diff(y, append=y[-1])
    length = np.hypot(dx, dy)
    nx = -dy / length
    ny = dx / length
    return nx, ny

def generate_road_boundaries(x, y, width):
    nx, ny = compute_normals(x, y)
    left_x = x + (width / 2) * nx
    left_y = y + (width / 2) * ny
    right_x = x - (width / 2) * nx
    right_y = y - (width / 2) * ny
    return left_x, left_y, right_x, right_y

def plot_data():
    
    ti = 0
    tf = len(heading_offset_data)
    t = np.arange(ti, tf) / frameRate

    plt.plot(t, heading_offset_data, 'b--')
    plt.xlabel('time[s]')
    plt.ylabel('heading[deg]')
    plt.title(f'heading data')
    plt.axhline(y=0, linestyle='--')
    plt.grid(True)
    plt.savefig(f'HEADING_DATA_{path_name}.png')
    plt.show()

    ti = 0
    tf = len(steering_current_data)
    t = np.arange(ti, tf) / frameRate
    plt.plot(t, steering_stanely_data, 'b--', label='Steering_pid_Data')
    plt.plot(t, steering_current_data, 'k--',label='Steering_cur_Data')
    plt.grid(True)
    plt.xlabel('time[s]')
    plt.ylabel('Steering[deg]')
    plt.title('Steering_Data')
    plt.legend()
    plt.savefig(f'Steering_Data_{path_name}.png')
    plt.show()

    ti = 0
    tf = len(lateral_offset_data)
    t = np.arange(ti, tf) / frameRate

    plt.plot(t, lateral_offset_data, 'b--')
    plt.xlabel('time[s]')
    plt.ylabel('lateral_offset[m]')
    plt.title(f'lateral_offset')
    plt.axhline(y=0, linestyle='--')
    plt.grid(True)
    plt.savefig(f'LATERAL_DATA_{path_name}.png')
    plt.show()



    plt.plot(position_data_x, position_data_y, 'bo', markersize =3, label='Position Data')
    plt.plot(global_x_data, global_y_data, 'r', markersize=3, label='Original Global Path')

    # plt.plot(left_x, left_y, 'k', markersize=5)
    # plt.plot(left_x, left_y, 'k.', markersize=5)
    # plt.plot(right_x, right_y, 'k', markersize=5)
    # plt.plot(right_x, right_y, 'k.', markersize=5)

    plt.grid(True)
    plt.xlabel('Global_path_x')
    plt.ylabel('Global_path_y')
    plt.title(f'Path_data')
    plt.legend()
    plt.axis('equal')
    plt.savefig(f'Path_data_{path_name}.png')
    plt.show()



    ti = 0
    tf = len(speed_current_data)
    t = np.arange(ti, tf) / frameRate

    plt.plot(t, speed_target_data, 'b-', label='Target Velocity')
    plt.plot(t, speed_current_data, 'r-', label='Current Velocity')
    plt.plot(t, speed_desired_data, 'g--', label='Desired Velocity')
    plt.xlabel('time[s]')
    plt.ylabel('Velocity [km/h]')
    plt.title('Velocity Tracking')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'VELOCITY_DATA_{path_name}.png')
    plt.show()




def calculateLOSS():
    if not lateral_offset_data:
        print("No lateral offset data to calculate.")
        return

    # 1. Lateral Offset 절댓값 처리 후 평균/표준편차
    lateral_offset_array = np.abs(np.array(lateral_offset_data))
    mean_offset = np.mean(lateral_offset_array)
    std_offset = np.std(lateral_offset_array)
    max_offset = np.max(lateral_offset_array)

    print(f"[Lateral Offset]")
    print(f"횡방향 오차 평균: {mean_offset:.4f} m")
    print(f"횡방향 오차 표준편차: {std_offset:.4f} ")
    print(f"횡방향 최대 오차: {max_offset:.4f} m")
    
    # 2. Steering RMS 계산 (조향 안정성 지표)
    if not steering_current_data:
        print("No steering data to calculate RMS.")
        return

    steering_array = np.array(steering_current_data)
    steering_rms = np.sqrt(np.mean(np.square(steering_array)))

    print(f"[Steering Stability]")
    print(f"Steering 입력 RMS: {steering_rms:.4f} deg")

    # 3. Heading RMS 계산
    if not heading_offset_data:
        print("No heading data to calculate RMS.")
        return

    heading_array = np.array(heading_offset_data)
    heading_rms = np.sqrt(np.mean(np.square(heading_array)))

    print(f"[Heading Stability]")
    print(f"Heading RMS: {heading_rms:.4f} deg")

    #4. 제동거리 계산
    if not speed_desired_data or not speed_current_data:
        print("No speed data to calculate braking distance.")
        return

    try:
        # 1. 제동 시작: desired velocity가 0이 되는 첫 시점
        brake_start_idx = next(i for i,v in enumerate(speed_desired_data) if v <= 0.1)

        # 2. 제동 종료: 그 이후 current velocity가 0이 되는 첫 시점
        brake_end_idx = next(i for i,v in enumerate(speed_current_data[brake_start_idx:]) if v <= 0.1) + brake_start_idx

        # 3. 제동 구간 속도 데이터 (km/h → m/s 변환)
        v_segment = np.array(speed_current_data[brake_start_idx:brake_end_idx]) / 3.6  

        # 4. 시간 간격
        dt = 1.0 / frameRate  

        # 5. 제동거리 = 속도 적분
        braking_distance = np.trapz(v_segment, dx=dt)

        print(f"[Braking Distance]")
        print(f"제동 시작 index: {brake_start_idx}, 제동 종료 index: {brake_end_idx}")
        print(f"제동거리: {braking_distance:.3f} m")

    except StopIteration:
        print("Braking event not found in speed data.")


if __name__ == "__main__":
    read_file()
    plot_data()
    calculateLOSS()