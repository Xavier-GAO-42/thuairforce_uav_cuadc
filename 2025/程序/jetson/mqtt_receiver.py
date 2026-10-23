# 负责通过mqtt和树莓派(图像采集)通信，从mqtt获取时间戳并从picture_nfs目录下读取对应的图片
# BROKER_IP使用本机自身IP 本机同时作为mqtt接收端与broker 需要在本机上运行mqtt服务器
import paho.mqtt.client as mqtt
import logging
import os
import multiprocessing
import signal
import threading
import time
import cv2

# 配置日志
logger = logging.getLogger(__name__)

# MQTT配置实例
# BROKER_IP = "192.168.144.25"  # 使用本机自身IP jetson同时作为mqtt接收端与broker 需要在本机上运行mqtt服务器
# PORT = 1883
# TOPIC = "filename"
def initializer(config):
    global BROKER_IP,PORT,TOPIC,COMMAND_TOPIC,do_raspi_check
    BROKER_IP = config.get_string("mqtt_receiver.BROKER_IP")
    PORT = config.get_int("mqtt_receiver.PORT")
    TOPIC = config.get_string("mqtt_receiver.TOPIC")
    COMMAND_TOPIC = config.get_string("mqtt_receiver.COMMAND_TOPIC")
    do_raspi_check = config.get_int("mqtt_receiver.do_raspi_check")

def mqtt_receiver(self_check_event,detect_start_event, image_queue, picture_queue, end_shutdown_event , detect_shutdown_event):
    """
    高效对齐位姿和图像时间戳，并启动YOLO识别进程
    
    参数:
        self_check_event: 自检事件，检查和树莓派之间的mqtt连接(收取到ready信号)与NFS连接(读取picture_nfs下名为ready的文件)，通过自检后set此事件
        detect_start_event: 开始侦察事件，由main.py在起飞之后set，之后mqtt_receiver向树莓派发送开始图像采集的mqtt信号
        image_queue: 传递图像时间戳和图像本身的队列 'timestamp' 'frame'，这里为这个队列的输入端
        picture_queue: 在detect阶段结束后，mqtt_receiver直接通过这个队列向rtsp进程发送图片(detect阶段中这个队列的输入端是yolo进程)
        end_shutdown_event: 飞机降落后被set，之后mqtt_receiver向树莓派发送终止图像采集信号，然后退出
        detect_shutdown_event: detect阶段结束后被set，将图像的输出队列由image_queue(给yolo)切换到picture_queue(给rtsp)
    """
    # 使用可变对象 + 线程锁保证安全
    global do_raspi_check
    state = {"lst_timestamp": 0}
    lock = threading.Lock()
    flag_raspi_ready = False
    if do_raspi_check == 0:
        flag_raspi_ready = True
    else:
        try:
            os.remove("./picture_nfs/ready")
        except:
            logger.info("remove nfs ready file failed")

    # 内部定义回调函数，可以访问外部函数的参数
    def on_connect(client, userdata, flags, rc):
        global TOPIC
        if rc == 0:
            logger.info(f"Connected to MQTT Broker subscribe TOPIC = {TOPIC}")
            client.subscribe(TOPIC)
        else:
            logger.error(f"Connection failed with code {rc}")

    def on_message(client, userdata, msg):
        # 检查是否已触发关闭
        if end_shutdown_event.is_set():
            return
        nonlocal flag_raspi_ready
        filename = msg.payload.decode("utf-8")
        try:
            filepath = f"./picture_nfs/{filename}"
            #print("msg from raspi: ",filename)
            if do_raspi_check == 1:
                if flag_raspi_ready == False:
                    logger.info(f"msg from raspi: {filename}")
                    if filename == 'ready':
                        logger.info("raspi send mqtt signal ready")
                        if os.path.isfile(filepath):
                            logger.info("raspi send NFS file ready")
                            flag_raspi_ready = True
                        else:
                            logger.info("[warning]NFS may fail")
                else:
                    if os.path.isfile(filepath) and os.path.getsize(filepath) > 0:
                        logger.info(f"Receiver mjpg: {filename}")
                        image_timestamp_ms = int(filename.replace(".jpg", ""))  # 直接替换掉扩展名
                        
                        with lock:  # 线程安全操作
                            if state["lst_timestamp"] < image_timestamp_ms:
                                # attention，由于摄像头安装角度问题，照片上方向对应飞机的后侧方向
                                frame = cv2.rotate(cv2.imread(filepath), cv2.ROTATE_180)
                                # 将数据放入队列
                                if not detect_shutdown_event.is_set(): # 在侦查阶段
                                    image_queue.put({'timestamp':image_timestamp_ms,'frame':frame})
                                else: # 在打击阶段，传输原始图像
                                    cv2.putText(frame, f"attack & land", 
                                            (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                                    picture_queue.put(frame)
                                state["lst_timestamp"] = image_timestamp_ms
                    else:
                        if filename == "ready":
                            pass
                        else:
                            logger.warning(f"File not found or empty: {filepath}")
            else:
                if os.path.isfile(filepath) and os.path.getsize(filepath) > 0:
                    logger.info(f"Receiver mjpg: {filename}")
                    image_timestamp_ms = int(filename.replace(".jpg", ""))  # 直接替换掉扩展名
                    
                    with lock:  # 线程安全操作
                        if state["lst_timestamp"] < image_timestamp_ms:
                            # 将数据放入队列
                            if not detect_shutdown_event.is_set(): # 在侦查阶段
                                image_queue.put({'timestamp':image_timestamp_ms,'frame':cv2.imread(filepath)})
                            else: # 在打击阶段，传输原始图像
                                frame = cv2.imread(filepath)
                                picture_queue.put(frame)
                            state["lst_timestamp"] = image_timestamp_ms
                else:
                    if filename == "ready":
                        pass
                    else:
                        logger.warning(f"File not found or empty: {filepath}")

                
        except ValueError:
            logger.error(f"Invalid timestamp in filename: {filename}")
        except Exception as e:
            logger.error(f"Error processing file: {str(e)}", exc_info=True)

    # 信号处理函数
    def handle_signal(signum, frame):
        logger.info(f"MQTT Received signal {signum}, initiating shutdown...")
        if not detect_shutdown_event.is_set():
            detect_shutdown_event.set()
        if not end_shutdown_event.is_set():
            end_shutdown_event.set()

    # 注册信号处理器
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)  # 同时处理Ctrl+C

    logger.info("Starting MQTT listener...")
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect(BROKER_IP, PORT, 60)
        # 启动非阻塞MQTT循环
        client.loop_start()
        logger.info("Starting MQTT listener... connected")
        # self_check 收到树莓派发送的mqtt信号start与raspi
        if do_raspi_check == 1:
            while flag_raspi_ready == False and not end_shutdown_event.is_set():
                logger.info(f"waiting raspi to be ready. {flag_raspi_ready}")
                time.sleep(1)
            if flag_raspi_ready == True:
                logger.info("raspi is ready")
            if end_shutdown_event.is_set():
                return
        
        # 标记自检完成
        self_check_event.set()
        time.sleep(3)
        while not detect_start_event.is_set():
            time.sleep(0.1)
            if end_shutdown_event.is_set():
                return
        
        # 给rpi5发送开始采集照片的信号
        logger.info("send start capture signal to raspi")
        client.publish(COMMAND_TOPIC, "capture start")

        
        # 主循环等待关闭信号
        while not end_shutdown_event.is_set():
            time.sleep(0.5)
            
        logger.info("Shutting down MQTT client...")
        
        # 优雅关闭MQTT连接
        client.publish(COMMAND_TOPIC, "capture stop")
        client.loop_stop()
        client.disconnect()
        logger.info("MQTT client disconnected successfully.")
        
    except Exception as e:
        logger.error(f"MQTT connection error: {str(e)}", exc_info=True)
    finally:
        # 确保资源清理
        if 'client' in locals() and client.is_connected():
            client.disconnect()
        logger.info("MQTT Receiver shutdown complete.")

# if __name__ == '__main__':
#     image_timestamp_queue=multiprocessing.Queue()
#     picture_queue=multiprocessing.Queue()
#     end_shutdown_event=multiprocessing.Event()
#     detect_shutdown_event=multiprocessing.Event()
#     # 启动mqtt监听进程 会自动结束
#     mqtt_receiver_process = multiprocessing.Process(
#         target=mqtt_receiver, 
#         args=(image_timestamp_queue, picture_queue, end_shutdown_event , detect_shutdown_event)
#     )
#     mqtt_receiver_process.daemon = True
#     mqtt_receiver_process.start()
#     time.sleep(5)
#     detect_shutdown_event.set()
#     #time.sleep(3)
