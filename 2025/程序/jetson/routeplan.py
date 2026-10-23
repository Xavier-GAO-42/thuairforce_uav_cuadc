# 各个阶段的航线规划，返回一个由Command对象构成的队列，missionlist
# 不使用vehicle对象，由navigate进程调用这个模块的函数后，将得到的missionlist用mission模块中的函数上传到飞机
# 这些函数主要靠ardupilot sitl模拟器测试，需要学会这个模拟器的基本使用方法、启动参数和mavproxy终端的基本指令
# attention: 比赛场地情况不同，这些函数也要相应修改，譬如禁飞区其实并非按照规则书划定
import math
import logging
import mission
import configuration
from dronekit import Command,mavutil

logger = logging.getLogger(__name__)

def initializer(config):
    global DEG,center_point,EDGE,ACC_R
    global LAUNCH_ALTITUDE,LAUNCH_SPEED
    global DETECT_ALTITUDE,DETECT_SPEED,DETECT_LOOP,DETECT_R
    global ATTACK_ALTITUDE,ATTACK_SPEED,ATTACK_LOOP,ATTACK_R,DROP_WP_DIS
    global RTL_ALTITUDE,RTL_R,LAND_WAYPOINT_FILE_PATH

    DEG = config.get_float("navigate.region_DEG")
    center_point = Point(config.get_float("navigate.centerpoint_lat"),
                        config.get_float("navigate.centerpoint_lon"))
    EDGE = config.get_int("navigate.region_EDGE")

    ACC_R = config.get_int("navigate.ACC_R")
    LAUNCH_ALTITUDE = config.get_int("navigate.LAUNCH_ALTITUDE")
    LAUNCH_SPEED = config.get_int("navigate.LAUNCH_SPEED")
    
    DETECT_ALTITUDE = config.get_int("navigate.DETECT_ALTITUDE")
    DETECT_SPEED = config.get_int("navigate.DETECT_SPEED")
    DETECT_LOOP = config.get_int("navigate.DETECT_LOOP")
    DETECT_R = config.get_int("navigate.DETECT_R")
    
    ATTACK_ALTITUDE = config.get_int("navigate.ATTACK_ALTITUDE")
    ATTACK_SPEED = config.get_int("navigate.ATTACK_SPEED")
    ATTACK_LOOP = config.get_int("navigate.ATTACK_LOOP")
    ATTACK_R = config.get_int("navigate.ATTACK_R")
    
    DROP_WP_DIS = config.get_int("navigate.DROP_WP_DIS")

    RTL_ALTITUDE = config.get_int("navigate.RTL_ALTITUDE")
    RTL_R = config.get_int("navigate.RTL_R")
    LAND_WAYPOINT_FILE_PATH = config.get_string("navigate.LAND_WAYPOINT_FILE_PATH")

class Point:
    def __init__(self,lat,lon):
        """先纬度，后经度"""
        self.lat = lat # 纬度
        self.lon = lon # 经度

    def move_point(self,dx,dy):
        """
        将这个点向东移动dx(m),向北移动dy(m)
        """
        a = 6378137.0
        lat_rad = math.radians(self.lat)
        m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_rad) + 1.175 * math.cos(4 * lat_rad)
        m_per_deg_lon = (math.pi * a / 180) * math.cos(lat_rad)  # a为地球长半轴
        d_lat = dy / m_per_deg_lat
        d_lon = dx / m_per_deg_lon
        return Point(self.lat+d_lat,self.lon+d_lon)

    def move_point_deg(self,r,deg):
        """
        将这个点向北东角deg(0°~360°)方向移动r(米)
        """
        dx = r*math.sin(math.radians(deg))
        dy = r*math.cos(math.radians(deg))
        return self.move_point(dx,dy)


    def four_points(self, deg, edge=60):
        """
        self: 打击区域中心点的经纬度坐标，类型为point
        deg: 北东角，单位为0~360度
        """
        deg = math.radians(deg-45) # 转化为弧度制
        dx = math.sin(deg) * edge * 0.5 * math.sqrt(2) # 东向距离
        dy = math.cos(deg) * edge * 0.5 * math.sqrt(2) # 北向距离

        left_up_point = self.move_point(dx,dy)
        right_up_point = self.move_point(dy,-dx)
        right_down_point = self.move_point(-dx,-dy)
        left_down_point = self.move_point(-dy,dx)

        return [left_up_point,right_up_point,right_down_point,left_down_point]

    def get_distance_metres(point1, point2):
        """
        获取两个经纬度坐标之间的距离
        输入  两个point类型数据(经纬度都为度-90~90,-180~180)
        输出  两个point之间的距离 单位为m
        """
        # 将十进制度数转换为弧度
        lon1, lat1, lon2, lat2 = map(math.radians, [point1.lon, point1.lat, point2.lon, point2.lat])
        # Haversine公式
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.asin(math.sqrt(a))
        # 地球半径（米）
        r = 6378137.0
        return c * r

    def get_turn_points(point1,point2,portions):
        """
        实际上是按照portions的比例分割point1与point2之间连成的线段
        返回len(portions)-1个Point类对象构成的List
        """
        total_protion = 0
        for portion in portions:
            total_protion += portion
        points = []
        d_lat = point2.lat - point1.lat
        d_lon = point2.lon - point1.lon
        pre_sum_protion = 0
        for portion in portions:
            pre_sum_protion+=portion
            points.append(Point(point1.lat+d_lat*pre_sum_protion/total_protion,point1.lon+d_lon*pre_sum_protion/total_protion))
        points.pop()
        return points

def calculate_dx_dy(point1, point2):
    """
    计算两个经纬坐标点之间的东向距离(dx)和北向距离(dy)
    
    参数:
    point1: Point类实例，第一个点
    point2: Point类实例，第二个点
    
    返回:
    dx: 东向距离，单位米 (正东方向为正)
    dy: 北向距离，单位米 (正北方向为正)
    """
    # 地球半径（米）
    R = 6371000  # 平均地球半径
    
    # 将经纬度转换为弧度
    lat1_rad = math.radians(point1.lat)
    lon1_rad = math.radians(point1.lon)
    lat2_rad = math.radians(point2.lat)
    lon2_rad = math.radians(point2.lon)
    
    # 计算纬度差和经度差（弧度）
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    # 计算北向距离（dy） - 沿着经线方向
    dy = R * dlat  # 简单的弧长公式，适用于小距离
    
    # 计算东向距离（dx） - 沿着纬线方向
    # 需要乘以平均纬度的余弦值来考虑纬度圈半径的变化
    mean_lat_rad = (lat1_rad + lat2_rad) / 2
    dx = R * math.cos(mean_lat_rad) * dlon
    
    return dx, dy
def get_deg(dx,dy):
    if dx == 0:
        if dy > 0:
            return 0
        else :
            return 180
    return ((math.atan2(dx,dy) / math.pi * 180.0) % 360.0 + 360.0) % 360.0
def abs(x):
    if x<0:
        return -x
    return x
def get_loiter_in_point(loiter_point_from, wise_tag1, loiter_point_to, wise_tag2, R):
    if wise_tag1 == wise_tag2:
        dx, dy = calculate_dx_dy(loiter_point_from, loiter_point_to)
        dx2, dy2 = dy, -dx
        len = math.sqrt(dx2**2 + dy2**2)
        ratio = R / len
        if wise_tag1 == -1:
            dx2, dy2 = dx2 * ratio, dy2 * ratio
            return loiter_point_to.move_point(dx2,dy2) 
        else :
            dx2, dy2 = -dx2 * ratio, -dy2 * ratio
            return loiter_point_to.move_point(dx2,dy2) 
    else:
        dx, dy = calculate_dx_dy(loiter_point_from, loiter_point_to)
        dx2, dy2 = dy, -dx
        len = math.sqrt(dx2**2 + dy2**2)
        deg1 = get_deg(dx,dy)
        deg2 = abs(math.asin(R / len * 2) / math.pi * 180.0)
        len2 = math.sqrt((len/2)**2 - R**2)
        res_point = loiter_point_from.move_point(dx/2,dy/2)
        if wise_tag1 == -1:
            return res_point.move_point_deg(len2, deg1 - deg2)
        else:
            return res_point.move_point_deg(len2, deg1 + deg2)

def land_route_plan():
    # 先在赛场中心盘旋降低高度到RTL_ALTITUDE Loiter_to_alt
    # global RTL_ALTITUDE,center_point
    # if turn_direction != -1:
    #     turn_direction = 1 # 顺时针
    turn_R = RTL_R
    loiter_point = center_point.move_point_deg(turn_R, (DEG + 90.0)%360.0)
    cmd_downalt = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,0,1,
                          0,-turn_R,0,1,loiter_point.lat,loiter_point.lon,RTL_ALTITUDE)
    # cmd_turn = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,0,1,
    #                    0.1,0,turn_R,1,loiter_point.lat,loiter_point.lon,RTL_ALTITUDE)
    # wp = center_point.move_point_deg(-50, DEG)
    # cmd_wp = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   0,1, 
    #                           0, 10, 0, 0, wp.lat, wp.lon, RTL_ALTITUDE)
    cmd_list = [cmd_downalt]
    cmd_list2 = mission.readmission(LAND_WAYPOINT_FILE_PATH)
    for cmd in cmd_list2:
        cmd_list.append(cmd)
    return cmd_list

def attack_route_plan(attack_point):
    """制订打击航线，返回Command类对象列表"""
    global ATTACK_ALTITUDE,ATTACK_SPEED,RTL_ALTITUDE,ATTACK_LOOP,DROP_WP_DIS
    # 攻击逻辑：在四个角点中选一个离目标最远的点盘旋一圈然后打击目标
    points_cal = center_point.four_points(DEG,EDGE+20)
    start_point = center_point
    max_dis = 0.0
    for point in points_cal:
        if Point.get_distance_metres(point,attack_point)>max_dis:
            max_dis = Point.get_distance_metres(point,attack_point)
            start_point = point
    # 盘旋降高度到相切位置进行打击
    cmd_downalt = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,0,1,
                          0,ATTACK_R,0,1,start_point.lat,start_point.lon,ATTACK_ALTITUDE)
    # cmd_setspeed= Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,1, 0,ATTACK_SPEED, 0, 0, 0, 0, 0)
    # 盘旋到相切位置进行打击
    cmd_turn = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,0,1,
                       0,0,ATTACK_R,1,start_point.lat,start_point.lon,ATTACK_ALTITUDE)
    # 航点直达攻击点
    cmd_attackpoint = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   0,1, 
                              0, DROP_WP_DIS, 0, 0, attack_point.lat, attack_point.lon, ATTACK_ALTITUDE)
    cmds = []
    cmds.append(cmd_downalt)
    cmds.append(cmd_attackpoint)
    # cmds.append(cmd_setspeed)
    for i in range(0,ATTACK_LOOP-1):
        cmds.append(cmd_turn)
        cmds.append(cmd_attackpoint)
    # 确保最后一次指令在执行的时候是正在靠近目标区域的，此时强制投弹一定至少能落在60*60的区域内
    return cmds

def detect_route_plan():
    """制订侦查航线，返回Command类对象列表"""
    global DEG,center_point,ACC_R
    global LAUNCH_ALTITUDE,LAUNCH_SPEED,DETECT_ALTITUDE,DETECT_SPEED,DETECT_LOOP

    # 获取打击区域的四个角点，以DEG方向(北东角)为正方向，从左上点顺时针方向依次排列 0,1,2,3
    points_cal = center_point.four_points(DEG,EDGE)
    # 把这四个点各自往上下(DEG方向正方向我称之为上)移动10m，以便飞机转弯后有时间调整姿态到平飞
    points_cal[0] = points_cal[0].move_point_deg(10,DEG)
    points_cal[1] = points_cal[1].move_point_deg(10,DEG)
    points_cal[2] = points_cal[2].move_point_deg(-10,DEG)
    points_cal[3] = points_cal[3].move_point_deg(-10,DEG)

    # 计算四个转弯盘旋的环绕中心
    up_turnpoints = Point.get_turn_points(points_cal[0],points_cal[1],[1,1,1])
    # up_turnpoints[0]:左上方 up_turnpoints[1]:右上方
    down_turnpoints = Point.get_turn_points(points_cal[2],points_cal[3],[1,1,1])
    # down_turnpoints[0]:右下方 down_turnpoints[1]:左下方

    first_point2 = down_turnpoints[0].move_point_deg(DETECT_R, (DEG + 90) % 360)
    first_point = first_point2.move_point_deg(-50, DEG)

    # 把转弯点排列好
    turn_points = [down_turnpoints[0],up_turnpoints[1],down_turnpoints[1],up_turnpoints[0]]
    # 设置转弯半径。绝对值为转弯半径，正数顺时针，负数逆时针
    turn_R = DETECT_R # 20

    # 构建任务列表
    cmd_list = []
    # 起飞任务
    cmd_takeoff = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 
                          20.0, 0, 0, 0, 0, 0, LAUNCH_ALTITUDE)
    cmd_list.append(cmd_takeoff)

    # 在起飞区域到侦查任务区域的过程中，设置巡航地速为LAUNCH_SPEED
    # cmd_setspeed_high  = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,1, 0,LAUNCH_SPEED, 0, 0, 0, 0, 0)
    # cmd_list.append(cmd_setspeed_high)

    # 前往入场航点edit!!!，同时爬升到DETECT_ALTITUDE高度
    cmd_firstpoint = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   0,1, 
                             0, 30, 0, 0, first_point.lat, first_point.lon, DETECT_ALTITUDE)
    cmd_list.append(cmd_firstpoint)
    cmd_firstpoint2 = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   0,1, 
                              0, 30, 0, 0, first_point2.lat, first_point2.lon, DETECT_ALTITUDE)
    cmd_list.append(cmd_firstpoint2)
    
    # 在侦查区域的任务重，设置巡航地速为DETECT_SPEED，低速进场
    # cmd_setspeed_low= Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,1, 0,DETECT_SPEED, 0, 0, 0, 0, 0)
    # cmd_list.append(cmd_setspeed_low)
    
    # 本来是想用dojump指令实现循环的，后来跑模拟器发现貌似有些问题，每个循环最后一个盘旋指令改出时间并没有等到航线与下一个航点相切
    # 所以直接重复加侦查命令。这样判断是否侦查阶段结束也比较方便
    point_index_list = [0, 1, 0, 3, 2, 3, 0, 1, 0, 3, 2, 3, 0]
    point_turn_list =  [-1,1,-1,-1, 1,-1,-1,-1,-1, 1, 1, 1,-1]
    point_index_len = 12
    for j in range(0,DETECT_LOOP):
        # 绘制一轮双八字航线
        for i in range(point_index_len):
            turn_point = turn_points[point_index_list[i]]
            turn_point_next = turn_points[point_index_list[i+1]]
            loiter_in_point = get_loiter_in_point(turn_point,point_turn_list[i],turn_point_next,point_turn_list[i+1],DETECT_R)
            cmd_point2 = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,1, 
                                 0, ACC_R, 0, 0, loiter_in_point.lat, loiter_in_point.lon, DETECT_ALTITUDE)
            cmd_list.append(cmd_point2)
            cmd_turn  = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,0,1,
                                0 ,0,turn_R*point_turn_list[i+1],1,turn_point_next.lat,turn_point_next.lon,DETECT_ALTITUDE)
            cmd_list.append(cmd_turn)
    #save_mission_list("./detect_qingneng.waypoints",cmd_list)
    return cmd_list

# if __name__ == '__main__':
    # 用来生成一些航点任务
    # config = configuration.Config('./config/config_qingneng_true.json')
    # initializer(config)
    # cmd_list = detect_route_plan()
    # mission.save_mission_list2('./waypoint/detect_qingneng_30mheight.waypoints',cmd_list)
    # cmd_list2 = attack_route_plan(Point(39.58581963,116.19874283))
    # mission.save_mission_list('./waypoint/attack_high.waypoints',cmd_list2)
    # cmd_list3 = land_route_plan()
    # mission.save_mission_list('./waypoint/land_new_testwaypoints',cmd_list3)