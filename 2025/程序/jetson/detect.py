# detect_receiver2 的聚类还需要测试
# def detect_aligner()接收pose_queue和image_timestamp_queue传来的数据
# 将两者对齐后将，调用yolo进程进行识别与解算
# yolo进程负责解算出靶标的gps位置与信息，将其返回到target_queue
# def detect_receiver(target_queue)接收靶标信息，进行聚类分析，在恰当的时候将解算出的打击位置返回到reture_queue，并设置detect_end_event
from sklearn.cluster import DBSCAN
from sklearn.metrics.pairwise import haversine_distances
import os
import multiprocessing
import time
import logging
import signal
import yolo
import numpy as np
# logging.basicConfig(
#     level=logging.INFO,
#     format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
# )
logger = logging.getLogger(__name__)

#mission=1 # 0识别图案   1识别数字
def initializer(config):
    global mission
    mission = config.get_int("mission")
    yolo.initializer(config)

def detect_receiver(target_queue, cluster_info_queue, result_queue, detect_shutdown_event):
    """
    实时目标接收与聚类分析系统
    :param target_queue: 目标检测队列，包含检测到的目标数据
    :param detect_shutdown_event: 关闭事件信号
    :param result_queue: 结果队列，用于返回最终目标坐标
    """
    global mission
    def graceful_shutdown(signum, frame):
        logger.info(f"Received signal {signum}, shutting down...")
        detect_shutdown_event.set()
    
    # 设置信号处理
    signal.signal(signal.SIGTERM, graceful_shutdown)
    signal.signal(signal.SIGINT, graceful_shutdown)

    logger.info("Target detection receiver started")
    
    # 聚类参数配置
    MIN_SAMPLES = 5         # 形成簇所需的最小样本数
    EPS_METERS = 7          # 聚类半径(米)
    MIN_CLUSTER_SIZE = 6    # 有效簇的最小点数
    CLUSTER_INTERVAL = 3    # 两次聚类的间隔时间
    #MAX_COLLECTION_SIZE = 100 # 最大收集点数
    
    collected_targets = []   # 存储收集到的目标数据
    last_cluster = time.time() # 记录开始时间
    
    try:
        while True:
            # 非阻塞收集目标数据
            while not target_queue.empty():
                target_data = target_queue.get(timeout=0.01)
                collected_targets.append(target_data)
                
                # 记录目标信息
                if mission == 1:
                    logger.info(f"[Number Target] Lat: {target_data['lat']:.6f}, Lon: {target_data['lon']:.6f}, "
                                f"ID: {target_data['pattern_id']}")
                else:
                    logger.info(f"[Graph Target] Lat: {target_data['lat']:.6f}, Lon: {target_data['lon']:.6f}, "
                                f"ID: {target_data['pattern_id']}")
            
            # 如果有足够数据，进行处理
            if detect_shutdown_event.is_set() or (len(collected_targets) >= MIN_CLUSTER_SIZE and time.time() - last_cluster >= CLUSTER_INTERVAL):
                logger.info(f"Starting clustering with {len(collected_targets)} targets...")
                # 提取经纬度数据
                points = np.array([[t['lat'], t['lon']] for t in collected_targets])
                
                # 转换为弧度（Haversine函数要求）
                points_rad = np.radians(points)
                
                # 计算Haversine距离矩阵（返回弧度距离）
                dist_rad = haversine_distances(points_rad)
                
                # 转换为米（地球半径6371000米） 这里近似估算一下即可
                dist_matrix = dist_rad * 6371000
                
                # 使用DBSCAN进行聚类
                db = DBSCAN(eps=EPS_METERS, min_samples=MIN_SAMPLES, metric="precomputed")
                labels = db.fit_predict(dist_matrix)
                
                # 统计各簇
                clusters = {}
                for i, label in enumerate(labels):
                    if label not in clusters:
                        clusters[label] = []
                    clusters[label].append(collected_targets[i])
                
                # 过滤有效簇（排除噪声点label=-1，且点数足够）
                valid_clusters = [cluster for label, cluster in clusters.items() 
                            if label != -1 and len(cluster) >= MIN_CLUSTER_SIZE]
                last_cluster = time.time()
                if not valid_clusters:
                    logger.warning("No valid clusters found. Waiting")
                    if detect_shutdown_event.is_set():
                        break
                    continue
                
                # 为每个簇确定代表性pattern_id（取众数）
                cluster_data = []
                for cluster in valid_clusters:
                    # 统计簇中所有pattern_id的出现频率
                    pattern_ids = [t['pattern_id'] for t in cluster]
                    unique_ids, counts = np.unique(pattern_ids, return_counts=True)
                    
                    # 选择出现次数最多的pattern_id作为该簇的代表
                    dominant_id = unique_ids[np.argmax(counts)]
                    
                    # 只保留与该代表pattern_id一致的点
                    consistent_points = [t for t in cluster if t['pattern_id'] == dominant_id]

                    if len(consistent_points) >= MIN_CLUSTER_SIZE:
                        # 计算簇中心
                        avg_lat = np.mean([t['lat'] for t in consistent_points])
                        avg_lon = np.mean([t['lon'] for t in consistent_points])
                    
                        logger.info(f"cluster data: dominant_id = {dominant_id}, consistent points num = {len(consistent_points)}, lat:{avg_lat}, lon:{avg_lon}")

                        cluster_data.append({
                            'points': consistent_points,
                            'pattern_id': dominant_id,
                            'size': len(consistent_points),
                            'center': (avg_lat, avg_lon)
                        })
                
                sorted_clusters = sorted(cluster_data, key=lambda x: x['size'], reverse=True)
                top_3_clusters = sorted_clusters[:3] # 因为已经知道总共有三个目标 我们认为识别次数最高的三个目标是真正的目标
                cluster_info_queue.put([t['pattern_id'] for t in top_3_clusters]) # 向rtsp推流进程发送目前的聚类id结果

                if len(cluster_data) < 3:
                    logger.info(f"Not detected all targets, detected {len(cluster_data)} targets")
                    if not detect_shutdown_event.is_set():
                        continue
                    ### 这里就是当侦查航线完成之后，还没有找到全部3个坐标，然后就随便调一个打
                    if len(cluster_data) >= 1:
                        logger.info("randomly choose a detected target")
                        top_3_clusters = sorted(top_3_clusters, key=lambda x: x['pattern_id'], reverse=True)# 把目标的数值或者价值从大到小排序
                        target_cluster = top_3_clusters[0]
                        avg_lat, avg_lon = target_cluster['center']
                        result_queue.put({"lat":avg_lat, "lon":avg_lon})
                    break

                top_3_clusters = sorted(top_3_clusters, key=lambda x: x['pattern_id'], reverse=False)# 把目标的数值或者价值从小到大排序
                
                if len(cluster_data)>3:
                    logger.info(f"[warning!!!] detected additional targets, detected {len(pattern_ids)} targets")
            
                # 根据任务类型选择目标簇
                if mission == 1:  
                    # 任务1: 数字目标 - 选择中位数的pattern_id
                    target_cluster = top_3_clusters[1] # 0 1 2 
                    target_id = target_cluster['pattern_id']
                    avg_lat, avg_lon = target_cluster['center']
                    
                    logger.info(f"Median pattern_id: {target_id}, selected target: {target_id}")
                else:  
                    # 任务0: 图形目标 - 选择最大pattern_id
                    target_cluster = top_3_clusters[2]
                    target_id = target_cluster['pattern_id']
                    avg_lat, avg_lon = target_cluster['center']
                    
                    logger.info(f"Max pattern_id target: {target_id}")
                            
                logger.info(f"Selected target at GPS: ({avg_lat:.6f}, {avg_lon:.6f})")
                
                ### 成功识别到三个靶，返回结果并触发关闭
                result_queue.put({"lat":avg_lat, "lon":avg_lon})
                detect_shutdown_event.set()
                logger.info("Target confirmed. Signaling shutdown.")
                break
    
    except Exception as e:
        logger.exception(f"Unexpected error in detection receiver: {str(e)}")
        detect_shutdown_event.set()
    finally:
        while not target_queue.empty():
            try:
                target_queue.get_nowait()
            except:
                break
        logger.info("Detection receiver process exiting gracefully")


def detect_receiver_test(target_queue, result_queue, detect_shutdown_event):
    """
    接收并处理目标识别结果，进行聚类分析后确定打击目标
    这个函数并没有写聚类，仅仅是用来测试。
    """
    global mission
    def graceful_shutdown(signum, frame):
        logger.info(f"Detect_Receiver received signal {signum}")
        detect_shutdown_event.set()
    
    signal.signal(signal.SIGTERM, graceful_shutdown)
    signal.signal(signal.SIGINT, graceful_shutdown)

    logger.info("Detect_Receiver start")
    
    # 参数配置
    collected_targets = []  # 存储收集到的目标
    
    while not detect_shutdown_event.is_set():
        # 收集目标数据
        if not target_queue.empty():
            target_data = target_queue.get()
            collected_targets.append(target_data)
            
            if mission == 1:
                logger.info(f"[Number Target] Lat: {target_data['lat']:.6f}, Lon: {target_data['lon']:.6f}, "
                            f"ID: {target_data['pattern_id']}")
            else:
                logger.info(f"[Graph Target] Lat: {target_data['lat']:.6f}, Lon: {target_data['lon']:.6f}, "
                            f"ID: {target_data['pattern_id']}")
        
        time.sleep(0.01)  # 避免CPU空转
    
    logger.info("Detect_Receiver process exiting gracefully")

def detect_aligner(detect_start_event,pose_queue, image_queue, target_queue, return_queue , detect_shutdown_event, picture_queue, cluster_info_queue):
    """
    高效对齐位姿和图像时间戳，并启动YOLO识别进程
    
    参数:
        detect_start_event: 开始侦察事件，由main.py在起飞之后set
        pose_queue: 从navigate.py传递位姿的队列，包含'timestamp'字段
        image_queue: 传递图像时间戳和图像本身的队列 'timestamp' 'frame'
        target_queue: YOLO进程返回目标识别结果的队列
        return_queue: detect成功后，将最终投放坐标输出到这个队列
        detect_shutdown_event: detect成功由detect.py设置，或侦察航点将要结束但还未侦察到目标时，由navigate.py set，detect_receiver将会从已有目标随机挑选一个放到return_queue中
        picture_queue: 由yolo进程放入被标注过的图片，传给rtsp进程输出到rtsp推流
        cluster_info_queue: 由detect_receiver线程放入当前聚类结果，传给rtsp进程，输出到rtsp推流的画面上
    """
    # 定义信号处理函数
    def graceful_shutdown(signum, frame):
        logger.info(f"Detect_Aligner Received signal {signum}, shutting down gracefully...")
        detect_shutdown_event.set()
        
    # 注册信号处理器
    signal.signal(signal.SIGTERM, graceful_shutdown)
    signal.signal(signal.SIGINT, graceful_shutdown)  # 同时处理Ctrl+C
    
    while not detect_start_event.is_set():
        time.sleep(1)
        if detect_shutdown_event.is_set():
            return
    try:
        global mission
        logger.info("Detect_Aligner start")

        # 启动靶标接收进程
        receiver_process = multiprocessing.Process(
            target=detect_receiver,
            args=(target_queue, cluster_info_queue, return_queue, detect_shutdown_event),
            daemon=True
        )
        receiver_process.start()
        logger.info("detect receiver process start")
        
        # 配置参数
        ALIGN_THRESHOLD = 10  # 时间戳对齐阈值(10ms)
        
        # 统计变量
        matched_count = 0
        pose_discard_count = 0
        image_discard_count = 0
        
        # 初始化状态
        current_image_ms = None
        current_pose = None

        while not detect_shutdown_event.is_set():
            # 获取初始图像时间戳
            if current_image_ms is None and not image_queue.empty():
                tmp = image_queue.get()
                current_image_ms = tmp['timestamp']
                frame = tmp['frame']
            
            # 获取初始位姿数据
            if current_pose is None and not pose_queue.empty():
                current_pose = pose_queue.get()
            
            # 检查是否有数据可处理
            if current_image_ms is None or current_pose is None:
                time.sleep(0.001)
                continue
            
            # 计算时间差
            diff = current_image_ms - current_pose['timestamp']
            
            # 处理时间戳不匹配的情况
            if diff > ALIGN_THRESHOLD:
                # 图像时间戳远大于位姿时间戳，丢弃旧位姿
                current_pose = None
                pose_discard_count += 1
                continue
            elif diff < -ALIGN_THRESHOLD:
                # 图像时间戳远小于位姿时间戳，丢弃旧图像
                current_image_ms = None
                image_discard_count += 1
                continue
            
            logger.info(f"upload run yolo mission {current_image_ms}.jpg")
            # 时间戳匹配成功，处理数据
            img_path = os.path.join("picture_nfs", f"{current_image_ms}.jpg")
            
            # 启动YOLO识别进程 提交进进程池
            #yolo_pool.apply_async(yolo.run_yolo_detection,
            #    args=(current_image_ms, current_pose, img_path, 
            #    target_queue, picture_queue, detect_shutdown_event)
            #    )
            yolo.run_yolo_detection(current_image_ms, current_pose, frame, 
                            target_queue, picture_queue, detect_shutdown_event)
            
            # 更新统计和状态
            matched_count += 1
            current_image_ms = None
            current_pose = None
            
            # 定期报告状态
            if matched_count % 10 == 0:
                logger.info(f"[Detect Aligner STATUS] Matched: {matched_count} | "
                      f"Discarded poses: {pose_discard_count} | "
                      f"Discarded images: {image_discard_count}")
    
    except Exception as e:
        logger.exception(f"Aligner exception: {str(e)}")
    finally:
        # 等待接收进程退出
        if receiver_process.is_alive():
            receiver_process.join(timeout=2.0)
            if receiver_process.is_alive():
                logger.warning("Detect_Aligner: Detect_Receiver process did not exit in time, terminating")
                receiver_process.terminate()

        logger.info("Detect_Aligner: Closing YOLO pool...")
        #yolo_pool.close()
        #yolo_pool.terminate()  # 立即终止工作进程
        #yolo_pool.join()
        logger.info("Detect_Aligner: YOLO pool closed.")

        logger.info("Detect_Aligner process exiting gracefully")

"""
def test_receiver():
    global mission
    mission = 1
    target_queue = multiprocessing.Queue() # 存储识别到的靶标x位置信息和pattern_id
    attack_pose_queue = multiprocessing.Queue() # 最终选择的目标经纬度位置信息。格式是dict ['lat']['lon']
    detect_shutdown_event = multiprocessing.Event()
    receiver_process = multiprocessing.Process(
        target=detect_receiver,
        args=(target_queue, attack_pose_queue, detect_shutdown_event),
        daemon=True
    )
    receiver_process.start()
    logger.info("detect receiver process start")

    # {'lat':'lon':'pattern_id'}
    for i in range(0,10):
        target_queue.put({
            'lat': 38.0,
            'lon':116.2,
            'pattern_id': 11
        })
        time.sleep(0.1)
        target_queue.put({
            'lat': 38.0,
            'lon':116.3,
            'pattern_id': 22
        })
        time.sleep(0.1)
        target_queue.put({
            'lat': 38.1,
            'lon':116.2,
            'pattern_id': 33
        })
        time.sleep(0.1)
    res = attack_pose_queue.get()
    print("get result")
    print(res)

def test_detect_aligner_shutdown():
    pose_queue = multiprocessing.Queue()
    image_timestamp_queue = multiprocessing.Queue()
    target_queue = multiprocessing.Queue()
    attack_pose_queue = multiprocessing.Queue()
    detect_shutdown_event= multiprocessing.Event()
    picture_queue= multiprocessing.Queue()
    detect_process = multiprocessing.Process(
        target=detect_aligner, 
        args=(pose_queue, image_timestamp_queue,target_queue,attack_pose_queue,detect_shutdown_event,picture_queue)
    )
    #detect_process.daemon = True # 守护进程无法开启子进程
    detect_process.start()    
    time.sleep(5)
    detect_shutdown_event.set()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        logger.info("main exit")
if __name__ == '__main__':
    test_receiver()
"""
