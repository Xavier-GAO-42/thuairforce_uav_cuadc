# 还需要测试
# 使用pyFilter库实现EKF扩展卡尔曼滤波（这个还需要调参）
# 我自己也写了一个LinearEKF的类，利用速度信息线性插值
# 之前还有一个版本是利用相邻两个GPS与姿态信息进行线性插值。不知道哪种更准确
# 还需要验证投蛋程序的准确性 attention here, 投弹存在严重问题

# https://github.com/dronekit/dronekit-python/issues/1132#issue-1046536695
# import collections # these can make dronekit run on python 3.12
# collections.MutableMapping = collections.abc.MutableMapping
# from dronekit import connect, VehicleMode, mavutil

from dronekit import connect,VehicleMode,mavutil
import threading
import time
import math
from collections import deque
import multiprocessing
import numpy as np
import logging
import queue
import mission
# import configuration
from routeplan import Point
import routeplan
import trajectory
from filterpy.kalman import ExtendedKalmanFilter

"""logging.basicConfig(
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    level=logging.INFO
)"""
logger = logging.getLogger(__name__)

# 全局状态变量
position_history = deque(maxlen=30)
attitude_history = deque(maxlen=60)
time_ref = {'unix_usec': 0, 'boot_ms': 0, 'local_time': 0}
last_processed_time = 0
vehicle = None
home_alt = None
measurement_queue = queue.PriorityQueue()  # 用于存储测量数据（时间戳，数据类型，数据）
do_handle = False # 用于控制mavlink回调函数是否执行

def initializer(config):
    global pose_fps,attack_lat_default,attack_lon_default,connection_string,drop_servo_channel
    pose_fps = config.get_int("navigate.pose_fps")
    attack_lat_default = config.get_float("navigate.attack_lat_default")
    attack_lon_default = config.get_float("navigate.attack_lon_default")
    connection_string = config.get_string("navigate.connection_string")
    routeplan.initializer(config)
    trajectory.initializer(config)
    drop_servo_channel = config.get_int("drop.servo_channel")

class LinearEKF:
    def __init__(self):
        self.x = np.zeros(9)
        self.R_earth = 6378137.0

    def update_position(self, z_pos):
        '''
        位置/速度测量更新
        z_pos: 测量向量 [纬度(rad), 经度(rad), 高度(m), vn, ve, vd] (6维)
        '''
        self.x[:6] = z_pos
    
    def update_attitude(self, z_att):
        '''
        姿态测量更新
        z_att: 测量向量 [roll, pitch, yaw] (rad) (3维)
        '''
        self.x[6:] = z_att
    def predict_with_dt(self,dt):
        # 解包状态
        lat, lon, alt, vn, ve, vd, roll, pitch, yaw = self.x
        
        # 位置更新 (考虑地球曲率)
        d_lat = vn * dt / self.R_earth
        d_lon = ve * dt / (self.R_earth * np.cos(lat))
        
        # 预测状态
        self.x = np.array([
            lat + d_lat,  # 纬度 (rad)
            lon + d_lon,  # 经度 (rad)
            alt - vd * dt,  # 高度 (m) vd向下为正？
            vn, ve, vd,  # 速度保持不变 (假设匀速)
            roll, pitch, yaw  # 姿态保持不变
        ])
        return self.x.copy()
    def get_state(self):
        '''获取当前状态'''
        return self.x.copy()
    
    def set_state(self, state):
        '''设置当前状态'''
        self.x = np.array(state)


# 完整的PoseEKF类实现 - 适配filterpy的ExtendedKalmanFilter接口
class PoseEKF:
    '''
    9维状态向量:
    0: 纬度 (rad)
    1: 经度 (rad)
    2: 高度 (m)
    3: 北向速度 (m/s)
    4: 东向速度 (m/s)
    5: 地向速度 (m/s) 向下为正
    6: 滚转角 (rad)
    7: 俯仰角 (rad)
    8: 航向角 (rad)
    '''
    
    def __init__(self):
        # 使用ExtendedKalmanFilter，状态维度9，测量维度9
        self.ekf = ExtendedKalmanFilter(dim_x=9, dim_z=9)
        self.R_earth = 6378137.0
        # 初始状态 - 设为全零，后续用实际测量初始化
        self.ekf.x = np.zeros(9)
        
        self.ekf.P = np.diag([
            # 位置方差 (纬度,经度,高度)
            (0.02 / 6378137.0)**2,   # 纬度方差 (对应1-2cm水平精度)
            (0.02 / 6378137.0)**2,   # 经度方差
            0.01**2,                 # 高度方差 (对应2-3cm垂直精度)
            # 速度方差
            0.05**2,                 # 北向速度方差
            0.05**2,                 # 东向速度方差
            0.1**2,                  # 地向速度方差 (垂直速度精度较低)
            # 姿态方差
            np.radians(0.3)**2,      # 滚转方差 (高精度IMU)
            np.radians(0.3)**2,      # 俯仰方差
            np.radians(1.0)**2       # 航向方差 (航向角精度通常较低)
        ])
        
        # 过程噪声协方差 (根据系统动态调整)
        # 基础过程噪声 (1秒)
        self.Q_base = np.array([
            1e-8, 1e-8, 0.001,
            0.01, 0.01, 0.02,
            np.radians(0.05)**2, np.radians(0.05)**2, np.radians(0.1)**2
        ])
        
        # 基础测量噪声协方差 (根据传感器精度调整)
        self.base_measurement_noise = np.diag([
            (0.01 / self.R_earth)**2,
            (0.01 / self.R_earth)**2,
            0.02**2,
            0.02**2,
            0.02**2,
            0.05**2,
            np.radians(0.2)**2,
            np.radians(0.2)**2,
            np.radians(0.5)**2
        ])
        
        # 设置状态转移函数和雅可比函数
        self.ekf.f = self.f  # 状态转移函数
        self.ekf.F_jacobian = self.F_jacobian  # 状态转移雅可比函数
        
        # 测量函数设置为单位矩阵（线性测量）
        self.ekf.H = np.eye(9)
        
        # 存储当前时间步长
        self._dt = 0.01  # 默认时间步长
    
    def set_dt(self, dt):
        """设置时间步长"""
        self._dt = dt
    
    def f(self, x, u=0):
        '''
        非线性状态转移函数
        x: 当前状态向量
        u: 控制输入 (未使用)
        返回: 预测状态向量
        '''
        # 使用类中存储的时间步长
        dt = self._dt
        
        # 解包状态
        lat, lon, alt, vn, ve, vd, roll, pitch, yaw = x
        
        # 位置更新 (考虑地球曲率)
        d_lat = vn * dt / self.R_earth
        d_lon = ve * dt / (self.R_earth * np.cos(lat))
        
        # 预测状态
        return np.array([
            lat + d_lat,  # 纬度 (rad)
            lon + d_lon,  # 经度 (rad)
            alt - vd * dt,  # 高度 (m)
            vn, ve, vd,  # 速度保持不变 (假设匀速)
            roll, pitch, yaw  # 姿态保持不变
        ])
    
    def F_jacobian(self, x, u=0):
        '''
        计算状态转移雅可比矩阵
        x: 当前状态向量
        u: 控制输入 (未使用)
        返回: 雅可比矩阵 (9x9)
        '''
        # 使用类中存储的时间步长
        dt = self._dt
        
        # 解包状态
        lat, _, _, vn, ve, _, _, _, _ = x
        
        # 初始化雅可比矩阵
        F = np.eye(9)
        
        # 纬度对北向速度的偏导
        F[0, 3] = dt / self.R_earth
        
        # 经度对东向速度的偏导
        F[1, 4] = dt / (self.R_earth * np.cos(lat))
        
        # 经度对纬度的偏导 (由于地球曲率)
        F[1, 0] = (ve * dt * np.sin(lat)) / (self.R_earth * np.cos(lat)**2)
        
        # 高度(向上为正)对地向速度(向下为正)的偏导
        F[2, 5] = - dt
        
        return F
    
    def predict_with_dt(self, dt):
        '''
        预测步骤 (带时间步长参数)
        dt: 时间步长 (秒)
        '''
        # 动态调整过程噪声
        self.set_dt(dt)
        # 调整Q矩阵：过程噪声与时间步长成正比
        scale_factors = np.array([
            dt, dt, dt,    # 位置
            dt, dt, dt,    # 速度
            dt, dt, dt     # 姿态
        ])
        self.ekf.Q = np.diag(self.Q_base * scale_factors)
        # 调用filterpy的预测方法
        self.ekf.predict()
    
    def update_position(self, z_pos):
        '''
        位置/速度测量更新
        z_pos: 测量向量 [纬度(rad), 经度(rad), 高度(m), vn, ve, vd] (6维)
        '''
        # 构建完整的9维测量向量
        z_full = np.zeros(9)
        z_full[:6] = z_pos
        z_full[6:] = self.ekf.x[6:]  # 姿态部分使用当前估计值
        
        # 创建测量噪声矩阵
        R = np.copy(self.base_measurement_noise)
        # 姿态部分使用非常大的噪声（相当于忽略姿态更新）
        R[6:, 6:] = 1e9 * np.eye(3)
        
        # 使用filterpy的更新方法
        self.ekf.update(z=z_full,HJacobian=self.F_jacobian,Hx=self.f, R=R)
    
    def update_attitude(self, z_att):
        '''
        姿态测量更新
        z_att: 测量向量 [roll, pitch, yaw] (rad) (3维)
        '''
        # 构建完整的9维测量向量
        z_full = np.zeros(9)
        z_full[:6] = self.ekf.x[:6]  # 位置部分使用当前估计值
        z_full[6:] = z_att
        
        # 创建测量噪声矩阵
        R = np.copy(self.base_measurement_noise)
        # 位置部分使用非常大的噪声（相当于忽略位置更新）
        R[:6, :6] = 1e9 * np.eye(6)
        
        # 使用filterpy的更新方法
        self.ekf.update(z=z_full,HJacobian=self.F_jacobian,Hx=self.f, R=R)
    
    def get_state(self):
        '''获取当前状态'''
        return self.ekf.x.copy()
    
    def set_state(self, state):
        '''设置当前状态'''
        self.ekf.x = np.array(state)
    
    # 以下属性提供对filterpy内部状态的直接访问
    @property
    def x(self):
        return self.ekf.x
    
    @property
    def P(self):
        return self.ekf.P
    
    @property
    def x_prior(self):
        return self.ekf.x_prior
    
    @property
    def P_prior(self):
        return self.ekf.P_prior
    
    @property
    def x_post(self):
        return self.ekf.x_post
    
    @property
    def P_post(self):
        return self.ekf.P_post
    
    @property
    def y(self):
        return self.ekf.y
    
    @property
    def K(self):
        return self.ekf.K
    
    @property
    def S(self):
        return self.ekf.S
    
    @property
    def z(self):
        return self.ekf.z
    
    @property
    def log_likelihood(self):
        return self.ekf.log_likelihood
    
    @property
    def likelihood(self):
        return self.ekf.likelihood
    
    @property
    def mahalanobis(self):
        return self.ekf.mahalanobis

def set_data_stream_rates():
    """设置更高频率的数据流"""
    global vehicle
    if vehicle is None:
        return
        
    try:
        # 位置数据频率 (Hz)
        vehicle.parameters['SR0_POSITION'] = 5
        
        # 姿态数据频率 (Hz)
        vehicle.parameters['SR0_EXTRA1'] = 10
        
        # 系统时间频率 (Hz)
        vehicle.parameters['SR0_EXTRA3'] = 5
        logger.info("数据流频率设置成功")
    except Exception as e:
        logger.exception(f"设置数据流频率时出错: {str(e)}")

def handle_system_time(_, name, msg):
    """处理系统时间消息"""
    global time_ref
    if msg.time_unix_usec > 0:  # 确保时间有效
        time_ref = {
            'unix_usec': msg.time_unix_usec,
            'boot_ms': msg.time_boot_ms,
            'local_time': time.time()
        }

def calculate_unix_time(time_boot_ms):
    """计算Unix时间戳（微秒）"""
    global time_ref
    if time_ref['unix_usec'] == 0:
        return 0
    return time_ref['unix_usec'] + (time_boot_ms - time_ref['boot_ms']) * 1000

def get_current_vehicle_time():
    """获取当前飞控时间(微秒)"""
    global time_ref
    if time_ref['unix_usec'] == 0:
        return 0  # 尚未建立时间基准
    
    # 计算自参考时间以来经过的时间
    elapsed_since_ref = time.time() - time_ref['local_time']
    
    # 假设飞控时间与本地时间线性相关
    current_unix_us = time_ref['unix_usec'] + elapsed_since_ref * 1e6
    
    return int(current_unix_us)

def handle_global_position(_, name, msg):
    """处理位置数据"""
    global do_handle
    if not do_handle:
        return
    unix_us = calculate_unix_time(msg.time_boot_ms)
    if unix_us == 0: # 在没有建立时间基准以前，不处理任何数据
        return
    
    # 将角度转换为弧度
    lat_rad = math.radians(msg.lat / 1.0e7)
    lon_rad = math.radians(msg.lon / 1.0e7)
    
    position_data = (
        unix_us,         # unix时间戳(微秒)
        lat_rad,         # 纬度(rad)
        lon_rad,         # 经度(rad)
        msg.alt / 1000.0,# 海拔高度 (m)
        msg.vx / 100.0,  # 北向速度 (m/s)
        msg.vy / 100.0,  # 东向速度 (m/s)
        msg.vz / 100.0   # 地向速度 (m/s)
    )
    #print("get pos data = ",position_data)
    measurement_queue.put((unix_us, 'position', position_data))

def handle_attitude(_, name, msg):
    """处理姿态数据"""
    global do_handle
    if not do_handle:
        return
    unix_us = calculate_unix_time(msg.time_boot_ms)
    if unix_us == 0: # 在没有建立时间基准以前，不处理任何数据
        return
    
    roll = msg.roll # 偏航角 弧度
    pitch = msg.pitch # 俯仰角 弧度
    yaw = msg.yaw % (2 * math.pi)  # 确保在0-2π范围内 航向角
    
    attitude_data = (unix_us, roll, pitch, yaw)
    measurement_queue.put((unix_us, 'attitude', attitude_data))

def run_ekf_interpolation(output_queue, ekf_shutdown_event):
    """主插值循环，使用EKF生成0.1秒间隔的位姿数据"""
    global time_ref, home_alt
    
    logger.info("EKF位姿估计已启动")
    
    # 初始化EKF
    ekf = LinearEKF() # attention 是否要用PoseEFK? 需要测试
    ekf_initialized = False
    last_ekf_time = None
    det_t = 1.0e6 / pose_fps  # 输出间隔 (微秒)
    next_output_time = 0 # 下一次的输出时间（微秒）
    initial_state = np.zeros(9)
    
    try:
        while not ekf_shutdown_event.is_set():
            current_time = get_current_vehicle_time()
            
            # 检查是否有足够的数据初始化EKF
            if not ekf_initialized:
                # 尝试获取第一个位置和姿态数据
                if measurement_queue.empty():
                    time.sleep(0.01)
                    continue
                    
                # 处理队列中的所有初始数据
                have_position = False
                have_attitude = False
                # 非阻塞获取数据，如果不行就整个再来一把
                while not measurement_queue.empty():
                    try:
                        timestamp, data_type, data = measurement_queue.get_nowait()
                    except queue.Empty:
                        break
                        
                    if last_ekf_time is None:
                        last_ekf_time = timestamp
                        next_output_time = (timestamp // det_t + 1) * det_t
                        
                    if data_type == 'position':
                        # 初始化位置状态
                        initial_state[0] = data[1]  # lat (rad)
                        initial_state[1] = data[2]  # lon (rad)
                        initial_state[2] = data[3]  # alt (m)
                        initial_state[3] = data[4]  # vn (m/s) 其实这里速度都可以为0，因为应该是放在地上的
                        initial_state[4] = data[5]  # ve (m/s)
                        initial_state[5] = data[6]  # vd (m/s) 向下为正
                        have_position = True
                    elif data_type == 'attitude':
                        # 初始化姿态状态
                        initial_state[6] = data[1]  # roll (rad)
                        initial_state[7] = data[2]  # pitch (rad)
                        initial_state[8] = data[3]  # yaw (rad)
                        have_attitude = True
                
                # 检查是否已初始化所有状态（移到while循环外部）
                if have_position and have_attitude and last_ekf_time:
                    # 调用状态设置函数
                    ekf.set_state(initial_state)
                    ekf_initialized = True
                    logger.info("EKF初始化完成")
                else:
                    logger.info(f"等待初始化数据... 位置:{have_position} 姿态:{have_attitude}")
                continue        
            
            # 如果已经初始化好了处理所有可用的测量数据
            if current_time >= next_output_time:
                # 处理所有在输出时间点之前的测量数据
                #while not measurement_queue.empty():
                while not ekf_shutdown_event.is_set():
                    if measurement_queue._qsize()>0:
                        # 查看下一个测量数据的时间戳
                        timestamp, data_type, data = measurement_queue.get_nowait()
                    else:
                        time.sleep(0.01)
                        continue
                    
                    # 直到有了测量数据时间在输出时间点之后，放回队列并跳出
                    if timestamp > next_output_time:
                        measurement_queue.put((timestamp, data_type, data))
                        break
                    
                    # 计算时间步长 (秒)
                    dt = (timestamp - last_ekf_time) * 1e-6
                    dt_us = timestamp - last_ekf_time
                    #logger.info(f"get a data {data_type} at {timestamp}, dt = {dt}")
                    # 预测步骤 先predict，从上一个last_ekf_time跳到time_stamp，然后再更新本时刻的数据
                    if dt_us >= 0:
                        ekf.predict_with_dt(dt=dt)
                        last_ekf_time = timestamp
                        # 更新步骤
                        if data_type == 'position':
                            z_pos = np.array(data[1:7])
                            ekf.update_position(z_pos)
                            #logger.info(f"ekf update with pos:{z_pos}")
                        elif data_type == 'attitude':
                            z_att = np.array(data[1:4])
                            ekf.update_attitude(z_att)
                            #logger.info(f"ekf update with att:{z_att}")
                    else:
                        logger.warning("EKF warning: data aborted")
                if ekf_shutdown_event.is_set():
                    break
                # 确保我们有最新的状态
                #if last_ekf_time is not None:
                # 计算从上次更新到输出时间的时间差
                dt_output = (next_output_time - last_ekf_time) * 1e-6
                if dt_output > 0:
                    ekf.predict_with_dt(dt=dt_output)
                    last_ekf_time = next_output_time
                
                # 获取当前状态
                state = ekf.get_state()
                
                # 构建位姿数据
                pose_data = {
                    'timestamp': int(next_output_time / 1000),  # 转换为毫秒
                    'lat': math.degrees(state[0]),  # 纬度 (度)，原来是弧度
                    'lon': math.degrees(state[1]),  # 经度 (度)，原来是弧度
                    'alt': state[2],                # 高度 (m)
                    'heading': state[8],  # 航向角 (弧度，0到2pi 北东角)
                    'roll': state[6],  # 滚转角 (弧度，正为右侧机翼向下)
                    'pitch': state[7], # 俯仰角 (弧度，正为抬头)
                    'yaw': state[8],   # 偏航角 (弧度，0到2pi 北东角)
                    'hei': state[2] - (home_alt if home_alt is not None else 0),      # 离地高度 (m)
                    'vn': state[3], # m/s 向北为正
                    've': state[4], # m/s 向东为正
                    'vd': state[5]  # m/s 向下为正
                }
                
                # 放入输出队列
                output_queue.put(pose_data)
                
                #logger.info(f"EKF输出时间: {next_output_time/1e6:.3f}s, 纬度: {pose_data['lat']:.6f}, 经度: {pose_data['lon']:.6f}, 高度: {pose_data['alt']:.1f}m late = {get_current_vehicle_time()-next_output_time/1e6:.5f}s")
                #logger.info(f"EKF输出时间: {next_output_time/1e6:.3f}s, late = {get_current_vehicle_time()/1e6-next_output_time/1e6:.5f}s")
                # 更新下一个输出时间
                next_output_time += det_t    
            # 控制循环频率
            time.sleep(0.001)
    except Exception as e:
        logger.exception(f"EKF线程崩溃: {e}")
    finally:
        if not ekf_shutdown_event.is_set():
            ekf_shutdown_event.set()
        logger.info("退出EKF插值")

def connect_to_vehicle(connection_string):
    """连接到飞控并设置消息监听器"""
    
    global vehicle, home_alt
    logger.info(f"正在连接到飞控: {connection_string}...")
    # vehicle = connect(connection_string, wait_ready = ['attitude', 'gps_0', 'armed', 'mode'], rate=50)
    vehicle = connect(connection_string, wait_ready = ['attitude', 'gps_0', 'armed', 'mode'])
    # vehicle = connect(connection_string, wait_ready = True)
    logger.info(f"连接成功! 系统状态: {vehicle.system_status.state}")
    
    logger.info("执行pre-arm check:")
    
    # attention检查可解锁状态 试飞的时候可能要注释掉。v5nano罗盘有问题
    # while not vehicle.is_armable:
    #     logger.info("等待飞控is_armable...")
    #     time.sleep(1)
    # 等待GPS锁定
    while vehicle.gps_0.fix_type < 3:
        logger.info(f"等待GPS锁定 (当前状态: {vehicle.gps_0.fix_type})...")
        time.sleep(1)
    # 先不要设置为AUTO模式
    if vehicle.mode.name == 'AUTO':
        vehicle.mode = VehicleMode("MANUAL")
        logger.info("已发送MANUAL模式指令")
        # 等待模式切换完成
        while not vehicle.mode.name == 'MANUAL':
            logger.info("当前模式: %s" % vehicle.mode.name)
            time.sleep(0.5)
        logger.info("无人机已退出AUTO模式并进入MANUAL模式")
    
    logger.info("清除残存航点 clean stale mission")
    clean_count = 0
    while clean_count < 3:
        try:
            if mission.clean_mission(vehicle):
                clean_count = 3
                break
        except Exception as e:
            logger.error(f"清除残存航点ERROR: {str(e)}")
            clean_count += 1

    time.sleep(3)
    # 设置更高频率的数据流
    # set_data_stream_rates()
    # 注册回调函数
    vehicle.add_message_listener('SYSTEM_TIME', handle_system_time)
    vehicle.add_message_listener('GLOBAL_POSITION_INT', handle_global_position)
    vehicle.add_message_listener('ATTITUDE', handle_attitude)
    
    while vehicle.home_location == None:
        logger.info(f"等待home_location... GPS: {vehicle.gps_0}")
        time.sleep(0.5)
    while vehicle.home_location == None:
        logger.info("等待home高度数据...")
        time.sleep(0.5)
    home_alt = vehicle.home_location.alt  # 单位m
    logger.info(f"获取家绝对海拔: {home_alt}m")

    while time_ref['unix_usec'] < 1754306200:  # 2025.8.4 北京时间晚上7点17分左右
        logger.info("等待有效的Unix时间戳...")
        time.sleep(1)
    logger.info(f"当前时间(Unix秒): {time_ref['unix_usec']/1e6:.3f}s")
    logger.info("飞控准备就绪")

def take_off():
    """
    解锁无人机引擎
    并进入auto模式执行起飞程序
    """
    
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    # 起飞任务检查
    if cmds.count > 1:
    # 获取索引1处的命令（第一个任务指令）
        first_cmd = cmds[0]
        if first_cmd.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            logger.info("起飞任务检查: success 第一个任务指令是起飞指令TAKEOFF")
        else:
            logger.info(f"起飞任务检查: fail 第一个任务指令不是起飞指令，而是：")
            for cmd in cmds:
                logger.info(cmd)
            return False
    else:
        logger.info("起飞任务检查: fail 任务列表中除了Home点外没有其他指令。")
        return False
    
    # 放一段音乐 喀秋莎第一句
    try:
        msg = vehicle.message_factory.play_tune_encode(
            0, 0, b"", 
            b"t200a4>d4c4d8.c8<b4a8g8a4d4."
        )
        vehicle.send_mavlink(msg)
        time.sleep(1)
        vehicle.send_mavlink(msg)
    except:
        logger.warning("music display 1 fail")
    # 设置drop channel pwm输出
    try:
        msg = vehicle.message_factory.command_long_encode(
            0, 0,           # 目标系统, 目标组件
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # 命令
            0,              # 确认
            drop_servo_channel,        # 伺服通道编号
            1100,      # PWM值(微秒)
            0, 0, 0, 0, 0   # 未使用的参数
        )
    except:
        logger.warning("send drop servo output channel is 1100 fail")

    ## attention: 这个在真实试飞的时候也务必删掉
    
    # global vehicle
    # if not vehicle.armed:
    #     logger.info("无人机未解锁，尝试解锁...")
    #     vehicle.armed = True
    #     while not vehicle.armed:
    #         logger.info("等待解锁...")
    #         time.sleep(1)
    #         vehicle.armed = True

    # attention: 设置AUTO模式 这个在实测时务必删掉，用遥控器手动设置为AUTO!!!
    
    # if not vehicle.mode.name == 'AUTO':
    #     vehicle.mode = VehicleMode("AUTO")
    #     logger.info("已发送AUTO模式指令")
    #     # 等待模式切换完成
    #     while not vehicle.mode.name == 'AUTO':
    #         logger.info("当前模式: %s" % vehicle.mode.name)
    #         time.sleep(0.5)
    #     logger.info("已进入AUTO模式")
    
    # attention: 地面测试的时候要删掉这个，否则进入不了侦查阶段
    # attention: 记得试飞前加上这一段
    logger.info("准备起飞，please set mode to AUTO, arm throttle")
    flag = False
    while vehicle.location.global_relative_frame.alt < 5:
        if vehicle.mode.name != 'AUTO' or not vehicle.armed:
            flag = False
            if not vehicle.mode.name != 'AUTO':
                logger.info("等待进入AUTO模式 当前模式: %s" % vehicle.mode.name)
            if not vehicle.armed:
                logger.info("等待解锁...")
            time.sleep(1)
        else:
            if not flag:
                try:
                    msg = vehicle.message_factory.play_tune_encode(
                        0, 0, b"", 
                        b"t200a4>d4c4d8.c8<b4a8g8a4d4."
                    )
                    vehicle.send_mavlink(msg)
                    time.sleep(3)
                except:
                    logger.info("music display 2 fail")
            flag = True
            logger.info(f"等待起飞高度达到5m，当前高度{vehicle.location.global_relative_frame.alt}m")
            time.sleep(1)
            # break
    
    return True

def drop_bomb():
    # 做一些事情进行投弹 譬如发送mavlink指令
    logger.info("bomb dropping!!!")
    global vehicle
    msg = vehicle.message_factory.command_long_encode(
        0, 0,           # 目标系统, 目标组件
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # 命令
        0,              # 确认
        drop_servo_channel,        # 伺服通道编号
        1900,      # PWM值(微秒)
        0, 0, 0, 0, 0   # 未使用的参数
    )
    vehicle.send_mavlink(msg)
    logger.info("bomb command 1 sent!!!")
    time.sleep(0.5)
    msg = vehicle.message_factory.command_long_encode(
        0, 0,           # 目标系统, 目标组件
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # 命令
        0,              # 确认
        drop_servo_channel,        # 伺服通道编号
        1100,      # PWM值(微秒)
        0, 0, 0, 0, 0   # 未使用的参数
    )
    vehicle.send_mavlink(msg)
    logger.info("bomb command 2 sent")

def land():
    global vehicle,land_success,land_wp_upload
    land_missionlist = routeplan.land_route_plan()
    mission.upload_mission_list(vehicle, land_missionlist)
    # mission.upload_mission_file(vehicle,LAND_WAYPOINT_FILE_PATH)
    while vehicle.mode.name == 'RTL':
        # 保证进入auto模式，因为飞控在航点执行完成的时候会自动切到RTL
        # 这么设计是因为确保假如要手动救机的时候，且到manual或stablize的时候不会又被切到auto。
        # 但作为代价，不能用RTL来救飞机了。
        vehicle.mode = VehicleMode("AUTO")
        logger.info(f"trying to enter AUTO, mode = {vehicle.mode.name}")
        time.sleep(0.5)
    land_wp_upload = True
    while vehicle.location.global_relative_frame.alt>1:
        time.sleep(3)
        logger.info(f"waiting to land, alt = {vehicle.location.global_relative_frame.alt}")
    logger.info("vehicle's relative alt < 1, land mission complete")
    land_success = True

def land_nowait():
    """强制上传降落航点，执行降落程序，不等待降落完成，用于navigate出错时"""
    global vehicle,land_success,land_wp_upload
    if vehicle.mode.name != 'RTL' and vehicle.mode.name != 'AUTO':
        # 飞手已经开始手动救飞机了
        logger.info("navigate ERROR: 飞手正在手动操纵飞机，land_nowait()上传航点取消")
        land_wp_upload = True
        return
    land_missionlist = routeplan.land_route_plan()
    mission.upload_mission_list(vehicle, land_missionlist)
    logger.info("navigate ERROR: 上传航点成功")
    while vehicle.mode.name == 'RTL':
        # 保证进入auto模式，因为飞控在航点执行完成的时候会自动切到RTL
        # 这么设计是因为确保假如要手动救机的时候，且到manual或stablize的时候不会又被切到auto。
        # 但作为代价，不能用RTL来救飞机了。
        vehicle.mode = VehicleMode("AUTO")
        logger.info(f"navigate ERROR: trying to enter AUTO, current mode = {vehicle.mode.name}")
        time.sleep(0.5)
    logger.info("navigate ERROR: enter AUTO success. good luck.")
    land_wp_upload = True

def abs(x):
    if x<0:
        return -x
    return x

def drop_and_land(attack_point,end_shutdown_event, check_R,pose_queue):
    global vehicle
    dropped = False # 是否已经投弹
    last_dis = -1
    while not dropped:
        
        # 这里是为了比较两种的差异
        pose = None
        while not pose_queue.empty():
            pose = pose_queue.get_nowait()
        dis = trajectory.calculate_bomb_trajectory(attack_point, vehicle)
        if pose:
            dis2 = trajectory.calculate_bomb_trajectory2(attack_point, pose)
            if dis2:
                logger.info(f"dis1 = {dis}, dis2 = {dis2}, det = {abs(dis - dis2)}")
            else:
                logger.info(f"calculate_bomb_trajectory2(EKF pose) wrong")
                logger.info(f"args: {attack_point},{pose}")
        
        if not dis:
            logger.info("calculate_bomb_trajectory wrong!")
            continue
        if dis < 20:
            if not (last_dis == dis):
                # 这是为了避免重复打印相同的dis。因为只有dronekit收到mavlink数据更新后，才能计算出不同的dis
                logger.info(f"drop compute dis is {dis}m")
                last_dis = dis
        if dis < check_R:
            drop_bomb()
            dropped = True
        elif vehicle.commands.next == len(vehicle.commands):# 正在最后一次尝试前往打击点
            logger.info("go to attack point last time")
            if dis < 30: # attention. 
                drop_bomb()
                logger.info(f"强制投弹, dis = {dis}")
                dropped = True
        elif vehicle.mode.name == 'RTL':
            drop_bomb()
            logger.info(f"强制投弹, dis = {dis}")
            dropped = True
        elif dis > 40:
            time.sleep(0.5)
    land()
    end_shutdown_event.set()
    logger.info("end_shutdown_event set in navigate.drop_and_land")

# 主进程
def navigate(self_check_event, start_take_off_event, take_off_event, detect_start_event, pose_queue, detect_shutdown_event, attack_pose_queue, end_shutdown_event):
    """
    self_check_event: 成功连上飞控、gps、获取到gps数据和home_alt后set这个事件
    start_take_off_event: mqtt自检和navigate自检都成功后，开始执行起飞任务(播放音乐，人工抛飞)
    take_off_event: 起飞后飞到5m后由navigate set，main.py收到此事件后将set detect_start_event
    detect_start_event: 等待这个事件is_set后开始插值，向输出pose_queue插值后的位姿时间戳数据
    pose_queue: 用于传递插值后的位姿数据，在detect阶段的接收者是detect.py，在attack阶段的接收者是navigate进程(drop_and_land函数)
    detect_shutdown_event: 用于控制detect进程的关闭。当detect航线将要结束时，通过set此事件强制detect进程从现有目标中选取一个目标进行打击，这个功能未经过严格测试(attention)
    attack_pose_queue: 用于传递打击目标位姿数据，传递一个dict 包含"lat"和"lon" 表示经纬度坐标，就是detect.py中的reture_queue
    end_shutdown_event: 用于控制程序的结束。这事件set后整个进程关闭。正常情况下由navigate进程在降落land()后设置
    """
    global vehicle,do_handle,land_success,land_wp_upload

    take_off_success = False
    land_success = False
    land_wp_upload = False

    # 定义ekf线程
    ekf_shutdown_event=multiprocessing.Event()
    ekf_thread = threading.Thread(
        target=run_ekf_interpolation, 
        args=(pose_queue,ekf_shutdown_event),
        daemon=True
    )

    try:
        connect_count = 0
        while True:
            connect_count += 1
            try:
                connect_to_vehicle(connection_string)
                break
            except Exception as e:
                logger.warning(e)
                logger.warning(f"tried {connect_count} times")
                time.sleep(3)
                pass
        
        self_check_event.set()
        """
        while not detect_start_event.is_set():
            time.sleep(0.5)
            if detect_shutdown_event.is_set() or end_shutdown_event.is_set():
                return
        """
        # 上载侦查航线航点
        detect_missionlist = routeplan.detect_route_plan()
        mission.upload_mission_list(vehicle, detect_missionlist)
        logger.info("upload detect mission plan")

        # 等待其他自检完成
        while not start_take_off_event.is_set():
            time.sleep(1)
            logger.info("waiting main to set start_take_off_event")
        ### 全部自检完成，准备起飞 ###
        
        ### 这里可以放一段音乐标记一下
        """
        msg = vehicle.message_factory.play_tune_encode(
            0, 0, b"", 
            b"t200a4>d4c4d8c8<b4a8g8a4d4"
        )# "t200a4>d4c4d8c8<b-4a8g8a4d4"
        vehicle.send_mavlink(msg)
        time.sleep(3)
        """
        if not take_off(): # 命令检查失败
            detect_shutdown_event.set()
            end_shutdown_event.set()
            logger.info("end_shutdown_event set in navigate because take off failed")
            #if ekf_thread.is_alive():
            #    ekf_thread.join(timeout=2)  # 等待EKF线程结束
            #    while ekf_thread.is_alive():
            #        logger.warning("EKF线程未能正常结束，强制退出")
            #        ekf_thread.join()  # 确保线程结束
            #    do_handle = False
            return
        else:
            take_off_success = True
        logger.info("takeoff successfully")
        take_off_event.set()
        
        ### 侦查阶段 ###
        while not detect_start_event.is_set():
            time.sleep(1)
            logger.info("waiting to start detect event")
        
        # 启动EKF线程
        do_handle = True
        ekf_thread.start()

        # 等待，直到detect进程结束
        while not detect_shutdown_event.is_set():
            time.sleep(1)
            
            # 监控队列状态
            logger.info(f"位姿队列大小: {pose_queue.qsize()}, 测量队列大小: {measurement_queue.qsize()}")
            if vehicle.commands.next == len(detect_missionlist):
                # 如果侦查航线到头了，那就直接停止detect进程
                detect_shutdown_event.set()
                break
        
        ### 打击阶段 ###
        time.sleep(0.5)
        # 确认打击目标坐标
        logger.info("detect进程已结束，开始打击阶段")
        attack_point_default = Point(attack_lat_default,attack_lon_default)
        attack_point = attack_point_default
        check_R = 25
        if not attack_pose_queue.empty():
            attack_pose = attack_pose_queue.get()
            logger.info(f"接收到打击目标位姿: {attack_pose}")
            attack_point = Point(attack_pose['lat'], attack_pose['lon'])
            check_R = 5
            # attention!!!! 实际飞的时候要加上这里
            if Point.get_distance_metres(attack_point_default,attack_point)>100:
                logger.info("attack point wrong, too far from default attack point!")
                logger.info(f"default: {attack_point_default}, receive: {attack_point}")
                attack_point = attack_point_default
                check_R = 25
        else:
            logger.info(f"no attack point received, attack default point")
        logger.info(f"finally we choose to attack point: {attack_point.lat},{attack_point.lon}")

        # 自主规划航线、上传航点
        logger.info("upload attack mission plan start")
        detect_missionlist = routeplan.attack_route_plan(attack_point)
        mission.upload_mission_list(vehicle, detect_missionlist)
        logger.info("upload attack mission plan done")
        while vehicle.mode.name == 'RTL':
            # 保证进入auto模式，因为飞控在航点执行完成的时候会自动切到RTL
            # 这么设计是因为确保假如要手动救机的时候，且到manual或stablize的时候不会又被切到auto。
            # 但作为代价，不能用RTL来救飞机了。
            vehicle.mode = VehicleMode("AUTO")
            logger.info(f"trying to enter AUTO, mode = {vehicle.mode.name}")
            time.sleep(0.5)
        logger.info(f"upload attack mission plan")
        # 进行打击，并在打击结束后上传返航航线
        drop_and_land(attack_point,end_shutdown_event,check_R,pose_queue)

        """
        while not end_shutdown_event.is_set():
            time.sleep(5)
            logger.info("打击并降落")
            if vehicle.commands.next == 22:
                logger.info("正在准备降落")
                logger.info("设置end_shutdown_event")
                end_shutdown_event.set()
                break
        """
        logger.info("navigate任务结束")

    except KeyboardInterrupt:
        logger.info("程序被用户手动终止")
    except Exception as e:
        logger.exception(f"navigate ERROR: 导航进程出错: {str(e)}")
    finally:
        # 保证出错的时候也要上传降落航线
        if take_off_success and not land_success and vehicle.location.global_relative_frame.alt > 5:
            land_nowait_count = 0
            while land_nowait_count < 3 and not land_wp_upload:
                try:
                    land_nowait()
                    logger.info("navigate ERROR: land_nowait() deal success")
                except Exception as e:
                    land_nowait_count += 1
                    logger.info(f"navigate ERROR, land_nowait() deal failed. error: {str(e)}")
        
        # 关闭ekf插值进程
        ekf_shutdown_event.set()
        do_handle = False
        time.sleep(3)
        while ekf_thread.is_alive():
            logger.warning("EKF线程未能正常结束，强制退出")
            ekf_thread.join()  # 确保线程结束
            time.sleep(1)
        if not detect_shutdown_event.is_set():
            detect_shutdown_event.set()
        if not end_shutdown_event.is_set():
            end_shutdown_event.set()
        # 关闭飞控连接
        if vehicle:
            vehicle.close()
            logger.info("飞控连接已关闭")
