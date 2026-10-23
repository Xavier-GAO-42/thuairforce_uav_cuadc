# 2025 树莓派图像采集系统

## 阅读路径

系统分工见[系统架构](../../航电设计/01-系统架构.md#architecture-5)，下面保留部署、自启动、接口与时间同步的原始记录；未展开的部分仍列为待补充项。

## 目录

- [1. 部署说明](#capture-1)
- [2. 开机自启动](#capture-2)
- [3. 硬件接口](#capture-3)
- [4. 图像回传](#capture-4)
- [5. 时间同步](#capture-5)

<a id="capture-1"></a>

## 1. 部署说明

不要改文件名

要用到摄像头、I2C-3总线连光传感器、usb(/dev/video0)连摄像头、PWM接口输出也要连摄像头（硬件快门）

系统镜像未随本资料包提供。

<a id="capture-2"></a>

## 2. 开机自启动

使用/etc/rc.local进行自启动
```bash
#!/bin/sh -e

# 延迟启动capture_nfs进程（等待系统完全启动）
sleep 1 && /usr/bin/python3 /home/thuair/CUADC/capture_nfs.py &

exit 0
```
这是一个简单的bash脚本，负责在系统启动结束后，自动启动拍照进程并挂到后台

<a id="capture-3"></a>

## 3. 硬件接口

### 3.1 I2C 配置

```bash
sudo raspi-conf
```
interface - i2c - enable
```bash
nano /boot/firmware/config.txt
```
增加两行
```bash
dtoverlay=i2c3-pi5,pins_6_7
```
你可以用
```bash 
dtoverlay -h i2c3-pi5
```
看一下关于这个设备覆盖层的信息
```bash
ls /boot/overlays/ # 支持的全部设备覆盖层
ls /boot/overlays/ | grep i2c # 与i2c interface 相关的设备覆盖层
```

### 3.2 PWM 输出与 GPIO 接线（原文待补充）



<a id="capture-4"></a>

## 4. 图像回传

### 4.1 NFS 图片传输（原文待补充）



### 4.2 MQTT 文件通知（原文待补充）



<a id="capture-5"></a>

## 5. 时间同步（原文待补充）


