# 控制rtsp服务器的开启与关闭
# 从picture_queue中读取图片并输出到rtsp服务器
# tested
import cv2
import numpy as np
import subprocess
import time
import multiprocessing
import signal
import logging

"""logging.basicConfig(
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    level=logging.INFO
)"""
logger = logging.getLogger(__name__)

def initializer(config):
    global rtsp_url,width,height,mediamtx_path,mediamtx_conf_path
    rtsp_url = config.get_string("rtsp.rtsp_url")
    width = config.get_int("rtsp.width")
    height = config.get_int("rtsp.height")
    mediamtx_path = config.get_string("rtsp.mediamtx_path")
    mediamtx_conf_path = config.get_string("rtsp.mediamtx_conf_path")

def rtsp_transmit(picture_queue,cluster_info_queue,shutdown_event):
    """
    picture_queue: detect阶段中这个队列的输入端是yolo进程，输入的是目标被标注plot过的图片
                   attack阶段中，这个队列的输入端是mqtt_receiver进程，输入的是原始图片
    shutdown_event: 其实就是navigate.py中的end_shutdown_event，该事件set后终止推流，退出进程
    """
    global rtsp_url,width,height

    # 定义信号处理函数
    def graceful_shutdown(signum, frame):
        #logger.info(f"Received signal {signum}, shutting down gracefully...")
        shutdown_event.set()    
    # 注册信号处理器
    signal.signal(signal.SIGTERM, graceful_shutdown)
    signal.signal(signal.SIGINT, graceful_shutdown)  # 同时处理Ctrl+C

    # 启动进程 rtsp服务器
    process = subprocess.Popen(
        [mediamtx_path, mediamtx_conf_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    try:
        # 启动FFmpeg进程（关键步骤）
        command = [
            '/usr/local/ffmpeg/bin/ffmpeg',
            '-y',  # 覆盖输出文件
            '-f', 'rawvideo',  # 输入为原始视频
            '-vcodec', 'rawvideo',  # 编解码器
            '-pix_fmt', 'bgr24',  # OpenCV默认BGR格式
            '-s', f'{width}x{height}',  # 分辨率
            #'-r', str(fps),  # 帧率
            '-i', '-',  # 从stdin读取
            '-c:v', 'libx264',  # 输出编码
            '-pix_fmt', 'yuv420p',  # 兼容格式
            '-preset', 'ultrafast',  # 快速编码
            '-f', 'rtsp',  # 输出格式
            rtsp_url
        ]

        # 启动FFmpeg进程并获取stdin管道
        ffmpeg_process = subprocess.Popen(
            command, 
            stdin=subprocess.PIPE,
            stderr=subprocess.DEVNULL  # 隐藏错误输出（调试时可移除）
        )

        logger.info("Start RTSP Stream")
        interval = 0.3 # 图传间隔
        last_time = time.time()
        info = []
        while not shutdown_event.is_set():
            while picture_queue.empty() and (not shutdown_event.is_set()):
                time.sleep(0.01)
            if shutdown_event.is_set():
                break
            # 从队列里取数据
            frame = picture_queue.get()
            # logger.info(f"get frame {frame.shape}")
            while not cluster_info_queue.empty():
                info = cluster_info_queue.get()
            if info:
                logger.info(f"get info {info}")
                frame = cv2.putText(frame, f"Detected cluster: {info}",
                        (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            else:
                frame = cv2.putText(frame, "Detected cluster: None",
                        (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            # 将帧写入管道 
            if time.time() - last_time > 0.2:
                ffmpeg_process.stdin.write(frame.tobytes())
                last_time = time.time()

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt rtsp Stream stopped")
    finally:
        # 清理资源
        logger.info("Stream stopping")
        ffmpeg_process.stdin.close()
        ffmpeg_process.terminate()
        ffmpeg_process.wait()
        process.terminate()
        # process.kill()

import configuration
# you can run ffplay rtsp://192.168.144.25:8554/mystream on other devices to check if it works well
if __name__ == '__main__':
    config_file_path="./config/config_qingneng_true.json"
    config = configuration.Config(config_file_path)
    initializer(config)

    pic_queue=multiprocessing.Queue()
    info_queue = multiprocessing.Queue()
    event = multiprocessing.Event()

    rtsp_process = multiprocessing.Process(target=rtsp_transmit, args=(pic_queue,info_queue,event))
    rtsp_process.daemon = True
    rtsp_process.start()

    fps = 10
    start_time = time.time()
    try:
        while True:
            # 创建动态帧（实际中替换为你的OpenCV图像数组）
            frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
            cv2.putText(frame, f"RTSP Stream: {time.time()}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            pic_queue.put(frame)
            info_queue.put([12,32,11])
            time.sleep(1 / fps)  # 控制帧率
    except KeyboardInterrupt:
        event.set()
