# 还需要测试哪种更精确，以及调整correct_t
# 提供两种计算落点与打击区域距离的函数
# calculate_bomb_trajectory 直接使用vehicle提供的参数
# calculate_bomb_trajectory2 使用EKF提供的位姿参数
import math
import time
from routeplan import Point

# 需要测试
g = 9.81  # 重力加速度 (m/s²)
drag_coefficient = 0.47  # 炸弹的空气阻力系数
bomb_mass = 0.35  # 炸弹的质量 (kg)
air_density = 1.225  # 空气密度 (kg/m^3)
# correction_t = 0 # 如果投弹延时比较大，可以增加这个数值 单位为秒

def initializer(config):
    global correction_t
    correction_t = config.get_float("drop.correction_t")

"""
dronekit的vehicle对象用法
# 姿态信息
print("姿态 (roll, pitch, yaw): %s" % vehicle.attitude)
print("航向: %s" % vehicle.heading)
# 位置信息
print("位置 (纬度, 经度, 高度): %s" % vehicle.location.global_frame)
print("相对高度: %s" % vehicle.location.global_relative_frame)
print("本地位置 (北, 东, 下): %s" % vehicle.location.local_frame)
"""
# 这个函数的时间要尽可能短
def calculate_bomb_trajectory(attack_point, vehicle):
    """
    attack_point: 将要打击的目标的经纬度
    vehicle: vehicle对象，用于读取当前飞行器的经纬度
    计算炸弹落地点和attack_point之间的距离
    返回的单位为m
    """
    # 定义常量
    global g, drag_coefficient, bomb_mass, air_density, correction_t
    cd = drag_coefficient  # 空气阻力系数
    m = bomb_mass          # 炸弹质量
    rho = air_density      # 空气密度
    A = 0.005              # 假设炸弹横截面积 (m^2)

    deg = math.degrees(vehicle.attitude.yaw)        # 偏航角（0=正北，π/2=正东）
    alt = vehicle.location.global_relative_frame.alt# 相对地面高度


    # 定义空气阻力
    def drag_force(vx, vy):
        v_squared = vx**2 + vy**2
        return 0.5 * cd * A * rho * v_squared

    # 初始化变量
    vx = vehicle.groundspeed  # 水平速度
    vy = vehicle.velocity[2]  # 竖直速度 向下为正
    x = 0   # 水平位移
    y = alt   # 竖直位移
    dt = 0.001  # 时间步长

    x += correction_t*vx
    
    try:
        # 迭代计算炸弹轨迹
        while y >= 0:
            # 计算阻力在水平和竖直方向上的分量
            fd = drag_force(vx, vy)
            fdx = fd * (vx / math.sqrt(vx**2 + vy**2))
            fdy = fd * (vy / math.sqrt(vx**2 + vy**2))

            # 更新水平和竖直速度
            ax = -fdx / m
            ay = -g - fdy / m
            vx += ax * dt
            vy += ay * dt

            # 更新位置
            x += vx * dt
            y += vy * dt

            # 如果炸弹已经落地，退出循环
            if y <= 0:
                break

        drop_point = Point(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon)
        drop_point = drop_point.move_point_deg(x,deg)
        return Point.get_distance_metres(drop_point,attack_point)
    except:
        return None
"""
pose_data = {
    'timestamp': int(next_output_time / 1000),  # 转换为毫秒
    'lat': math.degrees(state[0]),  # 纬度 (度)
    'lon': math.degrees(state[1]),  # 经度 (度)
    'alt': state[2],                # 高度 (m)
    'heading': math.degrees(state[8]) % 360,  # 航向角 (度)
    'roll': math.degrees(state[6]),  # 滚转角 (度)
    'pitch': math.degrees(state[7]), # 俯仰角 (度)
    'yaw': math.degrees(state[8]) % 360, # 偏航角 (度)
    'hei': state[2] - home_alt      # 离地高度 (m)
}
velocity[0][1][2]
"""
def calculate_bomb_trajectory2(attack_point, pose_data):
    """
    attack_point: 将要打击的目标的经纬度
    pose_data: 飞行器的位姿数据
    计算在pose_data情况下投弹，落点与attack_point之间的距离
    返回的单位为m
    """
    # 定义常量
    current_time = time.time()
    global g, drag_coefficient, bomb_mass, air_density, correction
    cd = drag_coefficient  # 空气阻力系数
    m = bomb_mass          # 炸弹质量
    rho = air_density      # 空气密度
    A = 0.005              # 假设炸弹横截面积 (m^2)

    deg = math.degrees(pose_data['yaw'])        # 偏航角（0=正北，π/2=正东）
    hei = pose_data['hei']# 相对地面高度


    # 定义空气阻力
    def drag_force(vx, vy):
        v_squared = vx**2 + vy**2
        return 0.5 * cd * A * rho * v_squared

    # 初始化变量
    vx = math.sqrt(pose_data['vn']**2+pose_data['ve']**2)  # 水平速度
    vy = -pose_data['vd']  # 竖直速度 向下为正
    x = 0   # 水平位移
    y = hei   # 竖直位移
    dt = 0.001  # 时间步长

    x += correction_t*vx
    
    try:
        # 迭代计算炸弹轨迹
        while y >= 0:
            # 计算阻力在水平和竖直方向上的分量
            fd = drag_force(vx, vy)
            fdx = fd * (vx / math.sqrt(vx**2 + vy**2))
            fdy = fd * (vy / math.sqrt(vx**2 + vy**2))

            # 更新水平和竖直速度
            ax = -fdx / m
            ay = -g - fdy / m
            vx += ax * dt
            vy += ay * dt

            # 更新位置
            x += vx * dt
            y += vy * dt

            # 如果炸弹已经落地，退出循环
            if y <= 0:
                break

        drop_point = Point(pose_data['lat'],pose_data['lon'])
        drop_point = drop_point.move_point_deg(x,deg)

        #print('time cost = ',time.time()-current_time)
        
        return Point.get_distance_metres(drop_point,attack_point)
    except:
        return None

"""
# {'timestamp': 1754631850400, 'lat': 39.58559268412783, 'lon': 116.1989053981502, 'alt': 44.33762708, 'heading': 331.0107134830795, 'roll': 0.40949726822065025, 'pitch': -3.593559317169169, 'yaw': 331.0107134830795, 'hei': 9.347629216230466, 'vn': 13.19, 've': -7.5, 'vd': 0.23}
if __name__ == '__main__':
    pose_data = {'timestamp': 1754631850400, 'lat': 39.58559268412783, 'lon': 116.1989053981502, 'alt': 44.33762708, 'heading': 331.0107134830795, 'roll': 0.40949726822065025, 'pitch': -3.593559317169169, 'yaw': 331.0107134830795, 'hei': 9.347629216230466, 'vn': 13.19, 've': -7.5, 'vd': 0.23}
    pointp = Point(39.58581963, 116.19874283)
    print(calculate_bomb_trajectory2(pointp,pose_data))
"""
