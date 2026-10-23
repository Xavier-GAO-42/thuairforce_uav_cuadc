# 这是一个可以用来分析目标识别结果的可视化分析器
# 用法: 
# python analyzer.py -c <log file path>
# e.g. python3 ./analyzer.py -c ./log_example
import re
import matplotlib.pyplot as plt
import numpy as np
# from matplotlib.colors import ListedColormap
from collections import defaultdict
# import matplotlib.cm as cm
import sys

plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def main():
    if len(sys.argv) < 3 or sys.argv[1] != '-c':
        print("用法: python script.py -c <log file path>")
        return
    
    log_file = sys.argv[2]
    # log_file = "./target_log_19"  # 替换为你的文件路径

    # 解析日志文件
    data_store = parse_log_file(log_file)
    # 可视化聚类情况
    visualize_clusters(data_store)

def parse_log_file(file_path):
    """
    从日志文件中解析目标信息
    :param file_path: 日志文件路径
    :return: 按pattern_id分组的坐标点字典
    """
    # 用于数字目标
    pattern = re.compile(
        r'\[Number Target\] Lat: ([\d.]+), Lon: ([\d.]+), ID: (\d+)'
    ) # 这是正则表达式
    # 用于图像目标
    # pattern = re.compile(
    #     r'Detect_Receiver: \[Graph Target\] GPS: \(([\d.]+), ([\d.]+)\), Number: (\d+)'
    # ) # 这是正则表达式
    
    data_store = defaultdict(list)
    
    with open(file_path, 'r') as file:
        for line in file:
            match = pattern.search(line)
            if match:
                # 提取并转换数据
                lat = float(match.group(1))
                lon = float(match.group(2))
                pattern_id = int(match.group(3))
                
                # 存储坐标点
                data_store[pattern_id].append((lat, lon))
    
    print(f"解析完成，共找到 {sum(len(points) for points in data_store.values())} 个目标点")
    print(f"包含 {len(data_store)} 种不同的pattern_id")
    
    return data_store

def visualize_clusters(data_store):
    """
    使用Matplotlib可视化聚类情况
    :param data_store: 按pattern_id分组的坐标点字典
    """
    # 创建图形和坐标轴
    plt.figure(figsize=(12, 10))
    ax = plt.subplot(111)
    
    # 设置颜色和标记样式
    colors = ['red', 'blue', 'green', 'purple', 'orange']
    markers = ['o', 's', '^', 'D', 'v']  # 圆形、方形、三角形、菱形、倒三角形
    
    # 确保不超过5种pattern_id
    pattern_ids = sorted(data_store.keys())
    if len(pattern_ids) > 5:
        print(f"警告：检测到超过5种pattern_id ({len(pattern_ids)}种)，仅显示前5种")
        for pattern_id in pattern_ids:
            print(f"patter_id: {pattern_id}")
        # pattern_ids = pattern_ids[:5]
    
    # 绘制每个pattern_id的点
    idx = 0
    for pid in pattern_ids:
        if len(data_store[pid])<10:
            continue
        points = data_store[pid]
        if not points:
            continue
        
        # 提取纬度和经度
        lats = [p[0] for p in points]
        lons = [p[1] for p in points]
        
        # 绘制散点图
        ax.scatter(
            lons, lats, 
            c=colors[idx], 
            marker=markers[idx],
            s=60,  # 点的大小
            alpha=0.7,  # 透明度
            edgecolors='w',  # 边缘颜色
            linewidths=0.5,  # 边缘线宽
            label=f'Pattern ID: {pid}'
        )
        
        # 计算并标注聚类中心
        if len(points) > 1:
            center_lat = np.mean(lats)
            center_lon = np.mean(lons)
            ax.scatter(
                center_lon, center_lat,
                c=colors[idx],
                marker='*',  # 星号表示中心点
                s=200,  # 更大的尺寸
                edgecolors='k',  # 黑色边缘
                linewidths=1
            )
            # 添加聚类中心标签
            ax.annotate(f'中心{pid}', 
                        (center_lon, center_lat),
                        textcoords="offset points", 
                        xytext=(0,10),
                        ha='center',
                        fontsize=9,
                        bbox=dict(boxstyle="round,pad=0.3", 
                                  fc=colors[idx], 
                                  alpha=0.2))
        idx = idx + 1
    
    # 设置坐标轴标签
    ax.set_xlabel('经度 (Longitude)', fontsize=12)
    ax.set_ylabel('纬度 (Latitude)', fontsize=12)
    
    # 添加标题
    plt.title('目标点位聚类分析', fontsize=14, pad=20)
    
    # 添加网格线
    ax.grid(True, linestyle='--', alpha=0.6)
    
    # 添加图例
    ax.legend(loc='best', fontsize=10, framealpha=0.9)
    
    # 添加比例尺
    min_lon = min([min(lons) for lons in [[p[1] for p in points] for points in data_store.values()] if lons])
    max_lon = max([max(lons) for lons in [[p[1] for p in points] for points in data_store.values()] if lons])
    lon_range = max_lon - min_lon
    
    # 在右下角添加比例尺
    # scale_length = lon_range * 0.2  # 比例尺长度为经度范围的20%
    # ax.plot([min_lon, min_lon + scale_length], 
    #         [min_lat := min([min(lats) for lats in [[p[0] for p in points] for points in data_store.values()] if lats]), 
    #          min_lat],
    #         color='black', linewidth=2)
    # ax.text(min_lon + scale_length/2, min_lat - 0.00005, 
    #         f'{scale_length:.6f}°', 
    #         ha='center', va='top', fontsize=9)
    
    # 添加文本说明
    total_points = sum(len(points) for points in data_store.values())
    plt.figtext(0.5, 0.01, 
               f'共可视化 {total_points} 个目标点 | 包含 {len(pattern_ids)} 种pattern_id',
               ha="center", fontsize=10, 
               bbox={"facecolor":"orange", "alpha":0.1, "pad":5})
    
    # 调整布局
    plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    
    # 保存和显示图像
    plt.savefig('target_clusters.png', dpi=300)
    print("可视化完成，图像已保存为 target_clusters.png")
    plt.show()

if __name__ == "__main__":
    main()