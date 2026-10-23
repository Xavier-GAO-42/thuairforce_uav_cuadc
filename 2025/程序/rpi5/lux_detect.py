# 通过I2C总线连接流明计
# 提供get_exposure_by_lux从lux数值计算曝光时间（单位并非ms，有一个对照表，参见相机文档）
import smbus2
import time
import logging

def get_exposure_by_lux(now_lux):
    res = int(40000/now_lux) # 加偏光 130000,不加50000
    if res == 0:
        res = 1
    if res > 500:
        res = 500
    return res

logger = logging.getLogger('Lux Detect')

# BH1750地址
BH1750_ADDR = 0x23  # 如果ADDR引脚接地则为0x23，接VCC则为0x5C

# 测量模式
CONTINUOUS_HIGH_RES_MODE = 0x10
CONTINUOUS_HIGH_RES_MODE_2 = 0x11
CONTINUOUS_LOW_RES_MODE = 0x13
ONE_TIME_HIGH_RES_MODE = 0x20
ONE_TIME_HIGH_RES_MODE_2 = 0x21
ONE_TIME_LOW_RES_MODE = 0x23

# 初始化I2C总线
bus = smbus2.SMBus(3)

def read_light_intensity():
    # 发送测量命令
    bus.write_byte(BH1750_ADDR, ONE_TIME_HIGH_RES_MODE)
    
    # 等待测量完成（最大180ms）
    time.sleep(0.2)
    
    # 读取2字节数据
    data = bus.read_i2c_block_data(BH1750_ADDR, 0, 2)
    
    # 转换数据为lux值
    light_level = (data[0] << 8) | data[1]
    light_level = light_level / 1.2  # 转换为lux
    
    return light_level

def get_avg_lux(get_times):
    count = 0
    avg_light = 0.0
    while count < get_times:
        try:
            light = read_light_intensity()
            avg_light += light
            logger.info(f"光照强度: {light:.2f} lux")
            # time.sleep(0.1)
            count += 1
        except Exception as e:
            logger.info(f"光照测量出错: Error: {e}")
            time.sleep(2)
    avg_light /= count
    return avg_light

def get_max_lux(get_times):
    count = 0
    max_light = 0.0
    while count < get_times:
        try:
            light = read_light_intensity()
            if max_light < light:
                max_light = light
            logger.info(f"光照强度: {light:.2f} lux")
            # time.sleep(0.1)
            count += 1
        except Exception as e:
            logger.info(f"光照测量出错: Error: {e}")
            time.sleep(2)
    return max_light

if __name__ == '__main__':
    print(get_max_lux(20))
