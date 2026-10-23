# 10hz output with software PWM
# 可能会因为负载而产生误差
import gpiozero
import time
import threading

# 配置参数
PULSE_PIN = 17  # 使用的GPIO引脚
FREQUENCY = 10  # 10Hz脉冲
PULSE_WIDTH = 0.001  # 脉冲宽度1ms

# 初始化GPIO
output = gpiozero.DigitalOutputDevice(PULSE_PIN)

def rising_edge_callback():
    """上升沿触发的回调函数"""
    # 在此添加需要执行的任务（确保快速执行）
    print(f"上升沿触发 @ {time.time():.6f}")

def pulse_generator():
    """精确时间脉冲生成器"""
    interval = 1.0 / FREQUENCY  # 0.1秒周期
    
    # 计算下一个对齐时间点（下一个整0.1秒）
    next_time = time.time() // interval * interval + interval - 0.0001
    
    while True:
        # 等待直到精确的时间点
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        
        # 生成上升沿
        output.on()
        rising_edge_callback()  # 执行回调
        
        # 维持脉冲宽度
        time.sleep(PULSE_WIDTH)
        
        # 生成下降沿
        output.off()
        
        # 计算下一个触发时间
        next_time += interval

# 创建并启动线程
pulse_thread = threading.Thread(target=pulse_generator, daemon=True)
pulse_thread.start()

# 主线程保持运行
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n程序终止")
    output.close()