import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import re
from datetime import datetime
import sys

plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def main():
    if len(sys.argv) < 3 or sys.argv[1] != '-c':
        print("用法: python script.py -c <log file path>")
        return
    
    log_file_path = sys.argv[2]
    # log_file_path = "log_test_navigate_1"  # 替换为您的日志文件路径
    
    print(f"正在从 {log_file_path} 读取数据...")
    timestamps, latitudes, longitudes, altitudes = parse_log_file(log_file_path)
    
    if timestamps:
        print(f"成功读取 {len(timestamps)} 个数据点")
        visualize_flight_data(timestamps, latitudes, longitudes, altitudes)
    else:
        print("未找到任何有效数据")

def parse_log_file(file_path):
    """
    从日志文件中提取所有包含"get pose"的数据行
    返回时间戳、纬度、经度和高度的列表
    """
    timestamps = []
    latitudes = []
    longitudes = []
    altitudes = []
    
    # 正则表达式匹配get pose行
    pattern = r'get pose: (\d+)ms, ([\d.]+), ([\d.]+), ([\d.]+)'
    count = 0
    with open(file_path, 'r', encoding='utf-8') as file:
        for line in file:
            if 'get pose' in line:
                match = re.search(pattern, line)
                if match:
                    count += 1
                    if count > 0:
                        try:
                            timestamp = int(match.group(1))
                            lat = float(match.group(2))
                            lon = float(match.group(3))
                            alt = float(match.group(4))
                            
                            timestamps.append(timestamp)
                            latitudes.append(lat)
                            longitudes.append(lon)
                            altitudes.append(alt)
                        except ValueError as e:
                            print(f"解析行时出错: {line.strip()}")
                            print(f"错误信息: {e}")
    
    return timestamps, latitudes, longitudes, altitudes

def visualize_flight_data(timestamps, latitudes, longitudes, altitudes):
    """
    可视化飞行数据
    """
    if not timestamps:
        print("未找到有效的飞行数据")
        return
    
    # 转换为相对时间（从第一个时间点开始）
    relative_time = [(ts - timestamps[0]) / 1000 for ts in timestamps]
    
    # 创建图形
    fig = plt.figure(figsize=(15, 10))
    
    # 1. 2D航线图
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(longitudes, latitudes, 'b-', marker='o', markersize=1, alpha=0.7)
    ax1.set_xlabel('经度')
    ax1.set_ylabel('纬度')
    ax1.set_title('无人机2D航线图')
    ax1.grid(True)
    
    # 添加起点和终点标记
    ax1.plot(longitudes[0], latitudes[0], 'go', markersize=8, label='起点')
    ax1.plot(longitudes[-1], latitudes[-1], 'ro', markersize=8, label='终点')
    ax1.legend()
    
    # 2. 高度变化图
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(relative_time, altitudes, 'r-', marker='s', markersize=4, alpha=0.7)
    ax2.set_xlabel('时间 (秒)')
    ax2.set_ylabel('相对高度 (米)')
    ax2.set_title('无人机高度变化')
    ax2.grid(True)
    
    # 3. 3D航线图
    ax3 = fig.add_subplot(2, 2, (3, 4), projection='3d')
    line = ax3.plot(longitudes, latitudes, altitudes, 'g-', marker='o', markersize=4, alpha=0.7)
    ax3.set_xlabel('经度')
    ax3.set_ylabel('纬度')
    ax3.set_zlabel('高度 (米)')
    ax3.set_title('无人机3D航线图')
    
    # 添加颜色映射以显示高度变化
    sc = ax3.scatter(longitudes, latitudes, altitudes, c=altitudes, cmap='viridis', s=20)
    plt.colorbar(sc, ax=ax3, label='高度 (米)')
    
    # 调整布局
    plt.tight_layout()
    plt.show()
    
    # 计算飞行统计数据
    total_distance = 0
    for i in range(1, len(latitudes)):
        # 简化的距离计算（使用平面近似）
        dx = (longitudes[i] - longitudes[i-1]) * 111320 * np.cos(np.radians(latitudes[i]))
        dy = (latitudes[i] - latitudes[i-1]) * 110574
        distance = np.sqrt(dx**2 + dy**2)
        total_distance += distance
    
    max_altitude = max(altitudes)
    min_altitude = min(altitudes)
    avg_altitude = np.mean(altitudes)
    flight_duration = relative_time[-1]
    
    print("="*50)
    print("飞行数据分析报告")
    print("="*50)
    print(f"数据点数量: {len(timestamps)}")
    print(f"飞行总距离: {total_distance:.2f} 米")
    print(f"最高高度: {max_altitude:.2f} 米")
    print(f"最低高度: {min_altitude:.2f} 米")
    print(f"平均高度: {avg_altitude:.2f} 米")
    print(f"飞行时间: {flight_duration:.2f} 秒")
    print(f"平均速度: {total_distance/flight_duration:.2f} 米/秒")
    
    # 计算高度变化统计
    altitude_changes = np.diff(altitudes)
    max_ascent = max(altitude_changes) if altitude_changes.any() else 0
    max_descent = min(altitude_changes) if altitude_changes.any() else 0
    
    print(f"最大上升率: {max_ascent:.2f} 米/秒")
    print(f"最大下降率: {max_descent:.2f} 米/秒")

    plt.savefig('navigate_analyzer.png', dpi=300)

if __name__ == "__main__":
    main()