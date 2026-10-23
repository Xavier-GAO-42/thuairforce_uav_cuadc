# 这些参数需要根据镜头调整
# 主要提供locate函数，反解靶标位姿
import numpy as np
import math

e2 = 0.00669437999014   # 地球参数（离心率的平方）
a = 6378137.0   # 地球参数 （长半轴长）
ground_height = 168.8   #飞行地点的海拔高度

def initializer(config):
    global fx,fy,centerx,centery
    fx = config.get_float("locate.fx")
    fy = config.get_float("locate.fy")
    centerx = config.get_float("locate.centerx")
    centery = config.get_float("locate.centery")

# 求矩阵的转置函数
def locate(pose_data, pic_x, pic_y):
    """
    反解靶标位姿函数
    
    参数:
        pose_data: yolo识别的图像对于的姿态数据 pose_data的定义见下方
        pic_x: 目标中心在图像中的x方向像素
        pic_y: 目标中心在图像中的y方向像素
    """
    global fx,fy,centerx,centery
    dro_pitch = pose_data['pitch']
    dro_yaw = pose_data['yaw']
    dro_roll = pose_data['roll']

    cy = math.cos(dro_yaw)
    sy = math.sin(dro_yaw)
    cp = math.cos(dro_pitch)
    sp = math.sin(dro_pitch)
    cr = math.cos(dro_roll)
    sr = math.sin(dro_roll)

    rotation_yaw = np.array([[cy, -sy, 0],
                             [sy, cy, 0],
                             [0, 0, 1]])
    rotation_pitch = np.array([[cp, 0, sp],
                               [0, 1, 0],
                               [-sp, 0, cp]])
    rotation_roll = np.array([[1, 0, 0],
                              [0, cr, -sr],
                              [0, sr, cr]])
    P = np.array([[0, -1, 0],
                  [1, 0, 0],
                  [0, 0, 1]])

    para = np.array([[1 / fx, 0, -centerx / fx],
                     [0, 1 / fy, -centery / fy],
                     [0, 0, 1]])

    imagevec = np.array([pic_x, pic_y, 1]) # 

    effect = np.dot(rotation_yaw, rotation_pitch)
    effect = np.dot(effect, rotation_roll)
    effect = np.dot(effect, P)
    effect = np.dot(effect, para)

    effectvec = np.dot(effect, imagevec)

    zc = pose_data['hei'] / effectvec[2]

    dro_north = zc * effectvec[0]
    dro_east = zc * effectvec[1]

#    unit_lat = 111194.926645
#    unit_lon = 85651.894194
#    dlat = dro_north/unit_lat
#    dlon = dro_east/unit_lon
    lat_rad = math.radians(pose_data['lat'])
    m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_rad) + 1.175 * math.cos(4 * lat_rad)
    m_per_deg_lon = (math.pi * a / 180) * math.cos(lat_rad)  # a为地球长半轴 使用了WGS84坐标系
    dlat = dro_north / m_per_deg_lat
    dlon = dro_east / m_per_deg_lon

    lat1 = pose_data['lat']+dlat
    lon1 = pose_data['lon']+dlon
    return lat1,lon1

"""
pose_data的定义
pose_data = {
    'timestamp': (int)(target_time/1000),  # Unix时间戳（ms）
    'lat': position[0],        # 纬度（度）
    'lon': position[1],        # 经度（度）
    'alt': position[2],        # 海拔高度（米）
    'heading': position[3],    # 航向角（弧度）
    'roll': attitude[0],       # 滚转角（弧度）正数值：右机翼下沉
    'pitch': attitude[1],      # 俯仰角（弧度）正数值：机头向上
    'yaw': attitude[2]         # 偏航角（弧度）0度为北，北东角，0到2pi
    'hei': position[2]-home_alt# 高度(米)
}
"""
# def test():
#     pose_data = {
#         'timestamp': 0,  # Unix时间戳（ms）
#         'lat': 30,        # 纬度（度）
#         'lon': 120,        # 经度（度）
#         'alt': 30,        # 海拔高度（米）
#         #'heading': math.radians(0),    # 航向角（弧度）0为正北 -pi到pi 顺时针为正方向
#         'roll': math.radians(0),       # 滚转角（弧度）正数值-右机翼下沉
#         'pitch': math.radians(30),      # 俯仰角（弧度）正数值-机头向上
#         'yaw': math.radians(0),         # 偏航角（弧度）0为正北 0到2*pi 顺时针为正方向
#         'hei': 30# 高度(米)
#     }
#     global fx,fy,centerx,centery

#     fx = fy = 1460
#     centerx = 960
#     centery = 540
    
#     lat,lon = locate(pose_data, 960,0)
#     print(f"res: {lat:.7f},{lon:.7f}")

# if __name__ == '__main__':
#     test()