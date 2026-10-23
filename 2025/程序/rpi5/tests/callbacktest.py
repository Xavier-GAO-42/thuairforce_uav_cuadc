#import numpy as np
from gpiozero import Button
from rpi_hardware_pwm import HardwarePWM
import time

PWM_FREQ=10
DUTY_CYCLE=30

def initPWM(FREQ,DUTY):
    global pwm
    pwm = HardwarePWM(pwm_channel=2, hz=FREQ, chip=0)
    pwm.start(DUTY) # full duty cycle
def setPWM(FREQ,DUTY):
    global pwm
    pwm.change_duty_cycle(DUTY)
    pwm.change_frequency(FREQ)
def stopPWM():
    global pwm
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
    if qwqcnt > 5:
        max_offset = max((abs(now - lst_time - 0.1)), max_offset)
    print(f"Rising edge at {now:.3f}s, timeoffset {time_off_set*1000:.3f}ms, gapoffset {abs(now-lst_time-0.1):.6f}")
    lst_time = now

    if qwqcnt % 40 == 0:
        realign_gpio()


def align_to_system_time():
    det=0.0005 + 0.00176 # s 0.001tiqian +0.0014
    current_time = time.time()
    interval=1/PWM_FREQ
    next_align_time = ((current_time+det)//interval+1)*interval-det  # 下一个0.1s的整数倍
    print("sleep",next_align_time-time.time())
    time.sleep(next_align_time-time.time())
    print("align to system time")

# 设置 GPIO 引脚（根据实际连接修改）
GPIO_PIN = 17  # 示例使用 GPIO17

# 创建 Button 对象
# 参数说明：
#   pull_up=False   : 启用下拉电阻（默认高电平为上升沿）
#   bounce_time=0.1 : 防抖动时间（秒）
#sensor = Button(GPIO_PIN, pull_up=False, bounce_time=0.001)
sensor = Button(GPIO_PIN, pull_up=False)

# 绑定上升沿事件到回调函数

def setup_gpio():
    """设置GPIO和PWM"""
    # 开启硬件PWM信号，占空比DUTY_CYCLE%
    
    sensor.when_pressed = pulse_callback
    align_to_system_time()
    initPWM(PWM_FREQ,DUTY_CYCLE)
    
    #logger.info("GPIO event detection enabled")
    print("GPIO event detection enabled")
    
    #logger.info(f"PWM started: {PWM_FREQ}Hz, {DUTY_CYCLE}% duty cycle")
    print(f"PWM started: {PWM_FREQ}Hz, {DUTY_CYCLE}% duty cycle")

def realign_gpio():
    stopPWM()
    align_to_system_time()
    initPWM(PWM_FREQ,DUTY_CYCLE)

def main():
    setup_gpio()
    try:
        while True:
            time.sleep(3)
#            realign_gpio()
    except KeyboardInterrupt:
        stopPWM()
    

if __name__ == "__main__":
    main()
