import detect
import logging
import multiprocessing
import time
import mqtt_receiver
import navigate_test
import rtsp
import configuration
import locate
import os

os.system("cd /home/thuair/CUADC") # 到达本项目所在目录

# attention: 读取配置
config_file_path="./config/config_qingneng_true.json"
config = configuration.Config(config_file_path)

# 配置日志
log_count = 1
log_file = f'./log/detect_test/log_test_{log_count}'
file_exists = os.path.isfile(log_file)
while file_exists: # Avoid same name
    log_count += 1
    log_file = f'./log/detect_test/log_test_{log_count}'
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

def main():
    logger.info(f"config all modules with {config_file_path}")
    # 调用各个模块的initializer
    detect.initializer(config)
    locate.initializer(config)
    mqtt_receiver.initializer(config)
    navigate_test.initializer(config)
    rtsp.initializer(config)
    logger.info("all module config initializer done")

    capture_selfcheck_event = multiprocessing.Event() # 树莓派连接，NFS与mqtt传输自检
    navigate_selfcheck_event = multiprocessing.Event() # 飞控连接，GPS自检
    start_take_off_event = multiprocessing.Event() # 开始起飞指令，当然之后还要等遥控器设置为auto与arm throttle
    take_off_event = multiprocessing.Event() # 起飞成功事件

    detect_start_event = multiprocessing.Event() # 开始所有detect相关进程

    detect_shutdown_event = multiprocessing.Event() # 停止侦查进程，它会让停止detect进程停止
    end_shutdown_event = multiprocessing.Event() # 其他所有进程最终共同的停止事件

    pose_queue = multiprocessing.Queue() # 插值得到的姿态信息
    image_queue = multiprocessing.Queue() # 存储图像时间戳，单位是ms
    target_queue = multiprocessing.Queue() # 存储识别到的靶标x位置信息和pattern_id
    attack_pose_queue = multiprocessing.Queue() # 最终选择的目标经纬度位置信息。格式是dict ['lat']['lon']
    picture_queue=multiprocessing.Queue() # 为rtsp传输图像帧。在detect结束前传输识别标记过的图片，在结束后直接由mqtt_receiver传输原始图片

    # 启动航线规划进程 会自动结束
    navigate_process = multiprocessing.Process(
        target=navigate_test.navigate, 
        args=(navigate_selfcheck_event,start_take_off_event,take_off_event,detect_start_event, pose_queue, 
              detect_shutdown_event, attack_pose_queue, end_shutdown_event)
    )
    navigate_process.daemon = True
    navigate_process.start()

    #self_check_event, start_take_off_event, take_off_event, detect_start_event, pose_queue, detect_shutdown_event, attack_pose_queue, end_shutdown_event

    # 启动rtsp推流进程 会自动结束
    rtsp_process = multiprocessing.Process(
        target=rtsp.rtsp_transmit, 
        args=(picture_queue,end_shutdown_event)
    )
    rtsp_process.daemon = True
    rtsp_process.start()

    # 启动mqtt监听进程 会自动结束
    mqtt_receiver_process = multiprocessing.Process(
        target=mqtt_receiver.mqtt_receiver, 
        args=(capture_selfcheck_event, detect_start_event,image_queue, picture_queue, end_shutdown_event , detect_shutdown_event)
    )
    mqtt_receiver_process.daemon = True # 守护进程
    mqtt_receiver_process.start()

    # 启动detect进程 会自动结束
    detect_process = multiprocessing.Process(
        target=detect.detect_aligner, 
        args=(detect_start_event,pose_queue, image_queue,target_queue,attack_pose_queue,detect_shutdown_event,picture_queue)
    )
    detect_process.start()


    try:
        ### 自检阶段 ###
        while True:
            if capture_selfcheck_event.is_set() and navigate_selfcheck_event.is_set():
                break
            else:
                time.sleep(1)
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
        while not detect_shutdown_event.is_set():
            time.sleep(1)
            # 监控队列状态
            logger.info(f"位姿队列大小: {pose_queue.qsize()}, 图像队列大小: {image_queue.qsize()}, 目标队列大小：{target_queue.qsize()}")

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

        # 检查一下所有进程是否都已经终止
        if mqtt_receiver_process.is_alive():
            mqtt_receiver_process.terminate()

        if navigate_process.is_alive():
            navigate_process.terminate()
        
        if detect_process.is_alive():
            mqtt_receiver_process.terminate()

        if rtsp_process.is_alive():
            rtsp_process.terminate()
        
        logger.info("所有进程已终止")

if __name__ == '__main__':
    main()