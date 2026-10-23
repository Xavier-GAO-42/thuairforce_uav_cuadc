# mqtt接收与发送器
import paho.mqtt.client as mqtt
import time
import logging
import threading

# 配置日志
logger = logging.getLogger("MQTTManager")

# 全局配置和状态
BROKER_IP = "192.168.144.25"  # 默认代理 IP
PORT = 1883                 # 默认端口
TOPIC = "filename"          # 默认主题
COMMAND_TOPIC = "command"

# 全局状态变量
client = None
is_connected = False
connection_lock = threading.Lock()
connection_retries = 0
MAX_RETRIES = 5
RETRY_DELAY = 5  # 秒
lst_command = "None"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info("Connected to MQTT Broker")
        client.subscribe(COMMAND_TOPIC)
    else:
        logger.error(f"Connection failed with code {rc}")

def on_message(client, userdata, msg):
    global lst_command
    command = msg.payload.decode("utf-8")
    if command == "capture start":
        lst_command = command
    elif command == "capture stop":
        lst_command = command
    else:
        logger.info("[warning] invalid command from controller, ignore")

def get_lst_command():
    global lst_command
    return lst_command

def mqtt_connect(broker_ip=None, port=None, max_retries=MAX_RETRIES, retry_delay=RETRY_DELAY):
    """
    连接到 MQTT 代理服务器
    
    参数:
        broker_ip (str): 代理服务器 IP 地址
        port (int): 代理服务器端口
        max_retries (int): 最大重试次数
        retry_delay (int): 重试间隔（秒）
    
    返回:
        bool: 连接是否成功
    """
    global client, is_connected, connection_retries, BROKER_IP, PORT
    
    # 使用默认值或传入参数
    broker_ip = broker_ip or BROKER_IP
    port = port or PORT
    
    with connection_lock:
        # 如果已连接，直接返回成功
        if is_connected:
            logger.info("MQTT 已连接，无需重复连接")
            return True
            
        # 创建新客户端（如果不存在）
        if client is None:
            client = mqtt.Client()
            client.on_connect = on_connect
            client.on_message = on_message
            client.on_disconnect = _on_disconnect
        
        # 尝试连接
        connection_retries = 0
        while connection_retries < max_retries:
            try:
                logger.info(f"尝试连接到 MQTT 代理: {broker_ip}:{port} (尝试 {connection_retries+1}/{max_retries})")
                client.connect(broker_ip, port, keepalive=60)
                
                # 启动网络循环
                client.loop_start()
                
                # 等待连接建立
                for _ in range(5):  # 最多等待 5 秒
                    if client.is_connected():
                        break
                    time.sleep(1)
                else:
                    raise ConnectionError("连接超时")
                
                is_connected = True
                logger.info("MQTT 连接成功")
                return True
                
            except (ConnectionRefusedError, TimeoutError, OSError) as e:
                connection_retries += 1
                logger.error(f"连接失败: {str(e)}")
                if connection_retries < max_retries:
                    time.sleep(retry_delay)
                else:
                    logger.error(f"所有 {max_retries} 次连接尝试均失败")
                    return False
            except Exception as e:
                logger.error(f"连接过程中发生意外错误: {str(e)}")
                return False

def mqtt_send(filename, topic=None, qos=0, retain=False):
    """
    通过 MQTT 发送消息
    
    参数:
        filename (str): 要发送的文件路径或消息内容
        topic (str): 消息主题
        qos (int): 服务质量等级 (0, 1, 2)
        retain (bool): 是否保留消息
    
    返回:
        bool: 消息是否发送成功
    """
    global client, is_connected
    
    # 使用默认主题或传入参数
    topic = topic or TOPIC
    
    if not is_connected:
        logger.warning("未连接到 MQTT，尝试重新连接...")
        if not mqtt_connect():
            logger.error("无法发送消息: 未建立连接")
            return False
    
    try:
        result = client.publish(topic, filename, qos=qos, retain=retain)
        
        # 对于 QoS > 0，等待消息确认
        if qos > 0:
            result.wait_for_publish(timeout=2.0)
            
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            logger.info(f"消息发送成功: {filename} [主题: {topic}]")
            return True
        else:
            logger.error(f"消息发送失败，错误码: {result.rc}")
            return False
            
    except Exception as e:
        logger.error(f"发送消息时出错: {str(e)}")
        # 标记连接断开
        is_connected = False
        return False

def mqtt_disconnect(force=False):
    """
    安全断开 MQTT 连接
    
    参数:
        force (bool): 是否强制立即断开
    """
    global client, is_connected
    
    with connection_lock:
        if client and is_connected:
            try:
                # 优雅断开
                if not force:
                    # 等待未完成的消息
                    client.loop(timeout=1.0)
                    # 发送断开消息
                    client.disconnect()
                    # 等待确认
                    for _ in range(3):
                        if not client.is_connected():
                            break
                        time.sleep(0.5)
                
                # 强制断开或优雅断开失败
                if force or client.is_connected():
                    client.loop_stop(force=force)
                
                is_connected = False
                logger.info("MQTT 已安全断开")
                return True
                
            except Exception as e:
                logger.error(f"断开连接时出错: {str(e)}")
                return False
        else:
            logger.info("MQTT 未连接，无需断开")
            return True

def _on_disconnect(client, userdata, rc):
    """MQTT 断开连接回调函数"""
    global is_connected
    is_connected = False
    
    if rc != 0:
        logger.warning(f"意外断开连接，返回码: {rc}")
    else:
        logger.info("已从服务器断开连接")

# 清理函数（用于程序退出）
def mqtt_cleanup():
    """清理 MQTT 资源"""
    mqtt_disconnect()
    global client
    client = None
    logger.info("MQTT 资源已清理")

# 示例用法
if __name__ == "__main__":
    #import atexit
    
    # 注册退出清理函数
    #atexit.register(mqtt_cleanup)
    
    # 连接测试
    if mqtt_connect():
        # 发送多条消息
        for i in range(5):
            filename = f"{i}.jpg"
            mqtt_send(filename)
            time.sleep(1)
        
        # 断开连接
        mqtt_disconnect()
