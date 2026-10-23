# 运行模拟器：Bash命令行
#   cd ardupilot/ArduPlane
#   ../Tools/autotest/sim_vehicle.py --console --map --custom-location=39.58546501,116.20102699,34.3,280
# mavproxy命令行
#   mode auto
#   arm throttle
# 这个程序要读取的配置：
#   "connection_string": "udpin:127.0.0.1:14550",字段要这样
#   当然从别的端口连接也许，需要用mavproxy再启用其他端口

# import detect
import logging
import multiprocessing
import time
# import mqtt_receiver
import navigate
# import rtsp
import configuration
# import locate
import os

os.system("cd /home/thuair/CUADC") # 到达本项目所在目录

# attention: 读取配置
config_file_path="./config/config_qingneng_true.json"
config = configuration.Config(config_file_path)

# 配置日志
log_count = 1
log_file = f'./log/navigate_test/log_test_navigate_{log_count}'
file_exists = os.path.isfile(log_file)
while file_exists: # Avoid same name
    log_count += 1
    log_file = f'./log/navigate_test/log_test_navigate_{log_count}'
    file_exists = os.path.isfile(log_file)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),  # 直接创建新实例
        logging.FileHandler(log_file)
    ]
)
logger = logging.getLogger(__name__)

def main_test_navigate():
    """
    这个代码是用来跑ardupilot-sitl仿真的
    只启动了navigate进程
    """
    logger.info(f"config all modules with {config_file_path}")
    # 调用各个模块的initializer
    navigate.initializer(config)
    logger.info("all module config initializer done")

    capture_selfcheck_event = multiprocessing.Event() # 树莓派连接，NFS与mqtt传输自检
    navigate_selfcheck_event = multiprocessing.Event() # 飞控连接，GPS自检
    start_take_off_event = multiprocessing.Event()
    take_off_event = multiprocessing.Event()
    detect_start_event = multiprocessing.Event() # 开始所有进程

    detect_shutdown_event = multiprocessing.Event()
    end_shutdown_event = multiprocessing.Event()

    pose_queue = multiprocessing.Queue()
    # image_queue = multiprocessing.Queue() # 存储图像时间戳，单位是ms
    target_queue = multiprocessing.Queue() # 存储识别到的靶标x位置信息
    attack_pose_queue = multiprocessing.Queue() # 最终选择的目标经纬度位置信息
    picture_queue=multiprocessing.Queue() # 为rtsp传输图像帧 opencv

    # 启动航线规划进程 会自动结束
    navigate_process = multiprocessing.Process(
        target=navigate.navigate, 
        args=(navigate_selfcheck_event,start_take_off_event,take_off_event,detect_start_event, pose_queue, 
              detect_shutdown_event, attack_pose_queue, end_shutdown_event)
    )
    navigate_process.daemon = True
    navigate_process.start()

    try:
        ### 自检与起飞 ###
        while True:
            if navigate_selfcheck_event.is_set():
                break
            else:
                time.sleep(3)
                logger.info("waiting for self check to be done")
        
        logger.info("self check all ready, going to take off")
        start_take_off_event.set()
        
        while not take_off_event.is_set():
            logger.info("waiting to take off")
            time.sleep(3)
        
        logger.info("set detect_start_event in main")
        detect_start_event.set()

        ### 侦查阶段 ###
        # 等待，直到detect进程自己结束
        count = 0
        current_time = time.time()
        while not detect_shutdown_event.is_set():
            time.sleep(0.5)
            while not pose_queue.empty():
                pose = pose_queue.get()
                count += 1
                # if count % 10 == 0:
                logger.info(f"get pose: {pose['timestamp']}ms, {pose['lat']}, {pose['lon']}, {pose['hei']}")
            
            if time.time()-current_time > 120:
                logger.info("结束模拟侦查阶段，开始打击7预设位置")
                attack_pose={"lat":39.58581963,"lon":116.19874283}
                attack_pose_queue.put(attack_pose)
                detect_shutdown_event.set()
                break

        ### 打击阶段 ###
        # 等待，直到navigate进程自己结束
        while not end_shutdown_event.is_set():
            time.sleep(1)
        
        logger.info("end shutdown event is set")

    except KeyboardInterrupt:
        logger.info("程序被用户手动终止")
        if not detect_shutdown_event.is_set():
            detect_shutdown_event.set()
        if not end_shutdown_event.is_set():
            end_shutdown_event.set()
            logger.info("end_shutdown_event set in main because KeyboardInterrupt")
    
    finally:
        time.sleep(5)
        if navigate_process.is_alive():
            navigate_process.terminate()
        logger.info("所有进程已终止")

if __name__ == '__main__':
    main_test_navigate()