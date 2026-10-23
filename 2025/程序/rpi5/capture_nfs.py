# 最新稳定版本的拍摄程序
import fcntl
import v4l2
import mmap
import os
import time
from gpiozero import Button,DigitalOutputDevice
from rpi_hardware_pwm import HardwarePWM
import threading
import queue
import logging
import signal
from concurrent.futures import ThreadPoolExecutor
import mqtt_sender
import lux_detect
import nfs_mount
import chronyc_check

# 确保执行路径在当前目录下
os.chdir("/home/thuair/CUADC")

SERVER_IP = "192.168.144.27"
CAPTURE_DIR = "./picture_nfs"  # 图像保存目录 nfs
SERVER_DIR = "/home/thuair/CUADC/picture_nfs"

log_filename = "./log_capture_nfs"
# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_filename),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('CameraSystem')

# 流明测量 确保上电后立即检测流明
now_lux = lux_detect.get_max_lux(20)
exposure_value = lux_detect.get_exposure_by_lux(now_lux)
logger.info(f"{now_lux} lux, exposure_value = {exposure_value}")

# 确保图片目录存在
if not os.path.exists(CAPTURE_DIR):
    os.makedirs(CAPTURE_DIR)
    logger.info(f"Created capture directory: {CAPTURE_DIR}")

# 挂载nfs
logger.info("trying to mount NFS")
while True:
    try:
        if nfs_mount.mount_nfs(SERVER_IP, SERVER_DIR, CAPTURE_DIR, "defaults"):
            logger.info("NFS mount success")
            break
        else:
            logger.info("NFS mount fail, retry")
    except Exception as e:
        logger.error(f"NFS mount error: {str(e)}")
    time.sleep(2)

# 确保时间校准
chronyc_check.chronyc_checker()

# 流明测量
# now_lux = lux_detect.get_avg_lux(20)
# exposure_value = lux_detect.get_exposure_by_lux(now_lux)
# logger.info(f"{now_lux} lux, exposure_value = {exposure_value}")

# 配置参数
GPIO_PIN = 18  # 硬件PWM引脚
PWM_FREQ = 10  # PWM频率 (Hz)
DUTY_CYCLE = 10  # 占空比 (%)
MAX_CAPTURE_WORKERS = 1  # 捕获线程数
MAX_STORAGE_WORKERS = 4  # 存储线程数
BUFFER_SIZE = 20  # 帧缓冲区大小
V4L2_BUFFER_COUNT = 4  # V4L2缓冲区数量

# 全局统计变量
capture_count = 0
storage_count = 0
missed_triggers = 0
last_trigger_time = time.time()
last_capture_end_time = time.time()

# 创建帧缓冲区
frame_buffer = queue.Queue(maxsize=BUFFER_SIZE)

# 退出事件
shutdown_event = threading.Event()

# 优化的V4L2Capture类 - 直接返回原始MJPG数据
class V4L2Capture:
    def __init__(self, device="/dev/video0", width=1920, height=1080, pixel_format='MJPG', buffer_count=6):
        self.fd = os.open(device, os.O_RDWR)
        self.width = width
        self.height = height
        self.pixel_format = self._get_fourcc(pixel_format)
        self.buffers = []
        self.buffer_count = buffer_count
        
        # 设置格式
        fmt = v4l2.v4l2_format()
        fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        fmt.fmt.pix.width = width
        fmt.fmt.pix.height = height
        fmt.fmt.pix.pixelformat = self.pixel_format
        fcntl.ioctl(self.fd, v4l2.VIDIOC_S_FMT, fmt)
        
        # 设置相机参数
        self._set_camera_controls()
        
        # 请求MMAP缓冲
        req = v4l2.v4l2_requestbuffers()
        req.count = buffer_count
        req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = v4l2.V4L2_MEMORY_MMAP
        fcntl.ioctl(self.fd, v4l2.VIDIOC_REQBUFS, req)
        
        # 映射缓冲
        for i in range(req.count):
            buf = v4l2.v4l2_buffer()
            buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = v4l2.V4L2_MEMORY_MMAP
            buf.index = i
            fcntl.ioctl(self.fd, v4l2.VIDIOC_QUERYBUF, buf)
            
            mm = mmap.mmap(
                self.fd, buf.length, 
                mmap.MAP_SHARED, 
                mmap.PROT_READ, 
                offset=buf.m.offset
            )
            self.buffers.append((mm, buf.length))
            fcntl.ioctl(self.fd, v4l2.VIDIOC_QBUF, buf)
        
        # 开始流
        buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.fd, v4l2.VIDIOC_STREAMON, buf_type)
    
    def _set_camera_controls(self):
        """设置相机控制参数"""
        # 设置对焦为手动模式
        focus_auto = v4l2.v4l2_control(id=v4l2.V4L2_CID_FOCUS_AUTO, value=0)
        fcntl.ioctl(self.fd, v4l2.VIDIOC_S_CTRL, focus_auto)

        # 设置对焦为无穷远
        focus_abs = v4l2.v4l2_control(id=v4l2.V4L2_CID_FOCUS_ABSOLUTE, value=0)
        fcntl.ioctl(self.fd, v4l2.VIDIOC_S_CTRL, focus_abs)
            
        # 设置白平衡为手动模式
        wb_auto = v4l2.v4l2_control(id=v4l2.V4L2_CID_AUTO_WHITE_BALANCE, value=0)
        fcntl.ioctl(self.fd, v4l2.VIDIOC_S_CTRL, wb_auto)
            
        # 设置白平衡温度6500K
        wb_temp = v4l2.v4l2_control(id=v4l2.V4L2_CID_WHITE_BALANCE_TEMPERATURE, value=6500)
        fcntl.ioctl(self.fd, v4l2.VIDIOC_S_CTRL, wb_temp)
            
        # 设置曝光为手动模式
        exposure_auto = v4l2.v4l2_control(id=v4l2.V4L2_CID_EXPOSURE_AUTO, value=v4l2.V4L2_EXPOSURE_MANUAL)
        fcntl.ioctl(self.fd, v4l2.VIDIOC_S_CTRL, exposure_auto)
            
        # 设置曝光时间 对照表参见该相机文档
        global exposure_value
        exposure_time = v4l2.v4l2_control(id=v4l2.V4L2_CID_EXPOSURE_ABSOLUTE, value=exposure_value)
        fcntl.ioctl(self.fd, v4l2.VIDIOC_S_CTRL, exposure_time)
    
        # 设置摄像头模式为硬件触发快门
        backlight_compensation = v4l2.v4l2_control(id=v4l2.V4L2_CID_BACKLIGHT_COMPENSATION, value=2)
        fcntl.ioctl(self.fd, v4l2.VIDIOC_S_CTRL, backlight_compensation)

    def _get_fourcc(self, fmt):
        formats = {
            'GREY': v4l2.v4l2_fourcc('G', 'R', 'E', 'Y'),
            'YUYV': v4l2.v4l2_fourcc('Y', 'U', 'Y', 'V'),
            'MJPG': v4l2.v4l2_fourcc('M', 'J', 'P', 'G')
        }
        return formats[fmt]
    
    def read_raw(self):
        """读取原始MJPG数据（不进行解码）"""
        # 出队缓冲
        buf = v4l2.v4l2_buffer()
        buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        buf.memory = v4l2.V4L2_MEMORY_MMAP
        fcntl.ioctl(self.fd, v4l2.VIDIOC_DQBUF, buf)
        
        # 直接访问内存映射
        mm, length = self.buffers[buf.index]
        mm.seek(0)
        frame_data = mm.read(buf.bytesused)
        
        # 重新入队缓冲
        fcntl.ioctl(self.fd, v4l2.VIDIOC_QBUF, buf)
        return frame_data
    
    def flush_buffers(self, count=10):
        """
        清空相机缓冲区
        :param count: 最大清空帧数 (防止无限循环)
        """
        logger.info(f"Flushing camera buffers (max {count} frames)")
        
        # 临时设置非阻塞模式
        original_flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, original_flags | os.O_NONBLOCK)
        
        flushed = 0
        try:
            while flushed < count:
                # 出队缓冲
                buf = v4l2.v4l2_buffer()
                buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
                buf.memory = v4l2.V4L2_MEMORY_MMAP
                
                try:
                    # 非阻塞尝试出队
                    fcntl.ioctl(self.fd, v4l2.VIDIOC_DQBUF, buf)
                except BlockingIOError:
                    # 无更多帧可读
                    break
                except OSError as e:
                    if e.errno == 11:  # EAGAIN
                        break
                    raise
                
                # 立即重新入队
                fcntl.ioctl(self.fd, v4l2.VIDIOC_QBUF, buf)
                flushed += 1
        finally:
            # 恢复原始阻塞模式
            fcntl.fcntl(self.fd, fcntl.F_SETFL, original_flags)
        
        logger.info(f"Flushed {flushed} frames from camera buffer")
        return flushed
    
    def release(self):
        """释放资源"""
        buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.fd, v4l2.VIDIOC_STREAMOFF, buf_type)
        for mm, _ in self.buffers:
            mm.close()
        os.close(self.fd)

# 创建相机锁和相机实例
camera_lock = threading.Lock()
camera = V4L2Capture(pixel_format='MJPG', buffer_count=V4L2_BUFFER_COUNT)
logger.info(f"Camera initialized with {V4L2_BUFFER_COUNT} buffers")

# 创建线程池
capture_executor = ThreadPoolExecutor(max_workers=MAX_CAPTURE_WORKERS)
storage_executor = ThreadPoolExecutor(max_workers=MAX_STORAGE_WORKERS)
storage_thread = None

def capture_frame(timestamp_ms):
    """捕获原始MJPG帧并放入缓冲区"""
    global last_capture_end_time, capture_count
    
    start_time = time.time()
    
    try:
        with camera_lock:
            frame_data = camera.read_raw()
        
        # 计算捕获时间
        capture_time = time.time() - start_time
        capture_count += 1
        
        logger.info(f"Capture #{capture_count} | Size: {len(frame_data)/1024:.1f}KB | Time: {capture_time*1000:.1f}ms")

        # 将原始数据和时戳放入缓冲区
        #if timestamp_ms>0:
        try:
            frame_buffer.put_nowait((frame_data, timestamp_ms))
            return True
        except queue.Full:
            logger.warning("Frame buffer full! Dropping frame")
            return False
        #else:
        #    logger.warning("Time offset too big!")
        

    except Exception as e:
        logger.error(f"Capture error: {str(e)}")
        return False
    finally:
        last_capture_end_time = time.time()

def store_raw_image(frame_data, timestamp_ms):
    """直接存储原始MJPG数据到文件"""
    global storage_count
    
    start_time = time.time()
    filepath = f"{CAPTURE_DIR}/{timestamp_ms}.jpg"
    filename = f"{timestamp_ms}.jpg"

    try:
        # 直接写入原始数据
        with open(filepath, 'wb') as f:
            f.write(frame_data)
        
        storage_time = time.time() - start_time
        storage_count += 1
        logger.info(f"Stored RAW #{storage_count} | Size: {len(frame_data)/1024:.1f}KB | Time: {storage_time*1000:.1f}ms | File: {filename}")
        
        # 发送mqtt信号
        mqtt_sender.mqtt_send(filename)
        
        return True
    except Exception as e:
        logger.error(f"Storage error: {str(e)}")
        return False


def storage_worker():
    """存储工作线程，从缓冲区获取原始数据并存储"""
    while not shutdown_event.is_set():
        try:
            # 从缓冲区获取原始数据
            frame_data, timestamp_ms = frame_buffer.get(timeout=1.0)
            
            if frame_data is None:  # 退出信号
                break
                
            # 提交存储任务
            storage_executor.submit(store_raw_image, frame_data, timestamp_ms)
            frame_buffer.task_done()
            
        except queue.Empty:
            # 定期检查退出条件
            continue
    logger.info("Storage worker exiting")

max_offset = 0
qwqcnt = 0
lst_time = time.time()

# 硬件定时器回调函数
def pulse_callback(gpio, level, tick):
    global lst_time, max_offset, qwqcnt, missed_triggers
    now = time.time()
    time_off_set = abs(now - ((float)(int(now * 10))) / 10)
    time_off_set = min(time_off_set, 0.1 - time_off_set)
    
    # 记录触发事件
    qwqcnt = qwqcnt + 1
    if qwqcnt > 5:
        max_offset = max((abs(now - lst_time - 0.1)), max_offset)
    logger.info(f"Rising edge at {now:.1f}s, timeoffset {time_off_set*1000:.3f}ms, gapoffset {abs(now-lst_time-0.1):.6f}")
    lst_time = now

    # 提交捕获任务
#    if time_off_set<0.005:
    timestamp_ms = int(now * 1000)
    try:
        logger.info(f"submit capture task at: {timestamp_ms}ms")
        capture_executor.submit(capture_frame, timestamp_ms)
    except Exception as e:
        missed_triggers += 1
        logger.error(f"Failed to submit capture task: {str(e)}")
#    else:
#        capture_executor.submit(capture_frame, -1)

# hardware pwm
pwm = None
def initPWM(FREQ,DUTY):
    global pwm
    pwm = HardwarePWM(pwm_channel=2, hz=FREQ, chip=0) # channel2控制的是GPIO_PIN=18
    pwm.start(DUTY) # full duty cycle
def setPWM(FREQ,DUTY):
    global pwm
    if pwm:
        pwm.change_duty_cycle(DUTY)
        pwm.change_frequency(FREQ)
def stopPWM():
    global pwm
    if pwm:
        pwm.stop()

max_offset = 0
qwqcnt = 0
lst_time = time.time()

# 硬件定时器回调函数
def pulse_callback():
    global lst_time, max_offset, qwqcnt, missed_triggers
    now = time.time()
    time_off_set = abs(now - ((float)(int(now * 10))) / 10)
    time_off_set = min(time_off_set, 0.1 - time_off_set)
    
    # 记录触发事件
    qwqcnt = qwqcnt + 1
    #if qwqcnt > 5:
    #    max_offset = max((abs(now - lst_time - 0.1)), max_offset)
    logger.info(f"pulse #{qwqcnt} at {now:.3f}s, timeoffset {time_off_set*1000:.3f}ms, gapoffset {abs(now-lst_time-0.1):.6f}")
    lst_time = now

    timestamp_ms = int(round(now , 2)*1000) # 四舍五入作为时间戳
    try:
        logger.info(f"submit capture task #{qwqcnt} at: {time.time()*1000}ms")
        capture_executor.submit(capture_frame, timestamp_ms)
    except Exception as e:
        missed_triggers += 1
        logger.error(f"Failed to submit capture task: {str(e)}")
    
    if qwqcnt % 40 == 0:
        realign_gpio()

def warm_up(WARM_UP_COUNT=5):
    """执行预热序列：发送脉冲并捕获图像"""
    logger.info(f"Starting camera warm-up sequence ({WARM_UP_COUNT} cycles)")
    initPWM(5,20)# 开始PWM脉冲
    for i in range(1, WARM_UP_COUNT + 1):
        # 捕获一帧图像
        camera.read_raw()
    stopPWM()
    time.sleep(3)
    camera.flush_buffers() # 清空v4l2缓冲区
    logger.info("Camera warm-up completed")

# 首次对齐
def align_to_system_time():
    det=0.0005 + 0.00176 # s 0.001tiqian +0.0014
    interval=1/PWM_FREQ
    next_align_time = ((time.time()+det)//interval+1)*interval-det  # 下一个0.1s的整数倍
    # print("sleep",next_align_time-time.time())
    time.sleep(next_align_time-time.time())
    logger.info("align to system time")


# 设置 GPIO 输入引脚
input_GPIO_PIN = 17  # 使用 GPIO17。硬件连接GPIO18(硬件PWM输出)与GPIO17

# 对GPIO17创建 Button 对象
#   pull_up=False   : 启用下拉电阻（默认高电平为上升沿）
sensor = Button(input_GPIO_PIN, pull_up=False)

def setup_gpio():
    """设置GPIO和PWM"""
    # 绑定pulse_callback为上升沿回调
    sensor.when_pressed = pulse_callback
    logger.info("GPIO event detection enabled")

    # 对齐系统时间并开启硬件PWM信号，占空比DUTY_CYCLE%
    align_to_system_time()
    initPWM(PWM_FREQ,DUTY_CYCLE)
    logger.info(f"PWM started: {PWM_FREQ}Hz, {DUTY_CYCLE}% duty cycle")

def realign_gpio():
    """再校准硬件PWM到系统时间"""
    stopPWM()
    align_to_system_time()
    initPWM(PWM_FREQ,DUTY_CYCLE)

def graceful_shutdown():
    """优雅关闭各类线程与释放资源"""
    global shutdown_event, capture_executor, storage_executor, camera, storage_thread
    
    # 设置关闭事件
    shutdown_event.set()

    # 清理GPIO
    stopPWM()
    logger.info("GPIO and PWM stopped")
    
    # 关闭捕获线程池
    if capture_executor is not None:
        logger.info("Shutting down capture executor...")
        capture_executor.shutdown(wait=False)
        logger.info("Capture executor shutdown")
        capture_executor = None
    
    # 处理存储线程
    if storage_thread is not None:
        if storage_thread.is_alive():
            try:
                # 清空缓冲区并发送退出信号
                while not frame_buffer.empty():
                    try:
                        frame_buffer.get_nowait()
                        frame_buffer.task_done()
                    except queue.Empty:
                        break
                frame_buffer.put((None, None))
                logger.info("Sent exit signal to storage worker")
                storage_thread.join(timeout=5.0)
                if storage_thread.is_alive():
                    logger.warning("Storage worker did not exit in time")
                else:
                    logger.info("Storage worker exited")
            except Exception as e:
                logger.error(f"Error sending exit signal: {str(e)}")
        storage_thread = None
    
    # 关闭存储线程池
    if storage_executor is not None:
        logger.info("Shutting down storage executor...")
        storage_executor.shutdown(wait=True)
        logger.info("Storage executor shutdown")
        storage_executor = None
    
    # 释放相机资源
    if camera is not None:
        camera.release()
        logger.info("Camera released")
        camera = None
    
    mqtt_sender.mqtt_disconnect()

def signal_handler(sig, frame):
    """处理中断信号"""
    logger.info(f"Received signal {sig}, initiating shutdown...")
    graceful_shutdown()
    logger.info("System shutdown complete")
    os._exit(0)

def force_exit(sig, frame):
    """强制退出处理"""
    logger.warning("Force shutdown requested! Exiting immediately")
    os._exit(1)

def main():
    """主函数"""
    global missed_triggers, storage_thread
    
    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGQUIT, force_exit)  # 第三次中断强制退出
    
    logger.info("Starting camera system with RAW MJPG storage")
    
    try:
        # 启动mqtt 连接
        while True:
            if mqtt_sender.mqtt_connect(broker_ip = SERVER_IP):
                logger.info("mqtt connect success")
                break
            else:
                logger.info("mqtt connect fail, retry")
                time.sleep(1)
        
        # 发送ready信号
        mqtt_sender.mqtt_send("ready")
        # os.system("cd ~/CUADC")
        os.system("touch ./picture_nfs/ready")

        lst_send_time = time.time()
        while mqtt_sender.get_lst_command() != "capture start":
            if time.time()-lst_send_time > 3:
                mqtt_sender.mqtt_send("ready")
                os.system("touch ./picture_nfs/ready")
                lst_send_time = time.time()
            time.sleep(0.5)

        # 启动存储工作线程
        storage_thread = threading.Thread(target=storage_worker, daemon=True)
        storage_thread.start()
        logger.info("Storage worker thread started")

        # 相机预热
        warm_up()

        # 开始脉冲
        setup_gpio()

        # 主监控循环
        last_report_time = time.time()

        while not shutdown_event.is_set():
            if mqtt_sender.get_lst_command() == "capture stop":
                break
            # 每5秒报告一次系统状态
            current_time = time.time()
            if current_time - last_report_time >= 5:
                buffer_size = frame_buffer.qsize()
                buffer_percent = (buffer_size / BUFFER_SIZE) * 100
                
                # 获取线程池队列大小
                capture_queue_size = capture_executor._work_queue.qsize()
                storage_queue_size = storage_executor._work_queue.qsize()
                
                logger.info(
                    f"Captures: {capture_count} | Storage: {storage_count} | "
                    f"Missed: {missed_triggers} | "
                    f"Buffer: {buffer_size}/{BUFFER_SIZE} ({buffer_percent:.1f}%) | "
                    f"Capture Queue: {capture_queue_size} | Storage Queue: {storage_queue_size} | "
                    f"Latency: {current_time - last_trigger_time:.3f}s"
                )
                
                # 重置计数器
                missed_triggers = 0
                last_report_time = current_time
                
            time.sleep(0.1)
            
    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}", exc_info=True)
    finally:
        if not shutdown_event.is_set():
            graceful_shutdown()
            logger.info("System shutdown complete")

if __name__ == "__main__":
    main()
    #while True:
    #    try:
    #        main()
    #    except KeyboardInterrupt:
    #        break
