connection_string = '/dev/ttyACM0,57600'  # 设备名,波特率
import time
import math
from dronekit import connect, VehicleMode, mavutil

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string) #, wait_ready=True)

print("Autopilot Firmware version: %s" % vehicle.version)
print("Global Location: %s" % vehicle.location.global_frame)

# Get some vehicle attributes (state)
print ("Get some vehicle attribute values:")
print (" GPS: %s" % vehicle.gps_0)
print (" Battery: %s" % vehicle.battery)
print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
print (" Is Armable?: %s" % vehicle.is_armable)
print (" System status: %s" % vehicle.system_status.state)
print (" Mode: %s" % vehicle.mode.name)    # settable)
try:
    while True:
        print ("yaw  : ",math.degrees(vehicle.attitude.yaw % (2*math.pi)))
        print ("roll : ",math.degrees(vehicle.attitude.roll))
        print ("pitch: ",math.degrees(vehicle.attitude.pitch))
        time.sleep(1)
except:
    print("interrupt")
#while not vehicle.is_armable:
#    print ("waiting aramble")
#    time.sleep(3)

# drop_servo_channel = 9
# msg = vehicle.message_factory.command_long_encode(
#     0, 0,           # 目标系统, 目标组件
#     mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # 命令
#     0,              # 确认
#     drop_servo_channel,        # 伺服通道编号
#     1900,      # PWM值(微秒)
#     0, 0, 0, 0, 0   # 未使用的参数
# )
# vehicle.send_mavlink(msg)
# print("bomb command 1 sent!!!")
# time.sleep(0.5)
# msg = vehicle.message_factory.command_long_encode(
#     0, 0,           # 目标系统, 目标组件
#     mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # 命令
#     0,              # 确认
#     drop_servo_channel,        # 伺服通道编号
#     1100,      # PWM值(微秒)
#     0, 0, 0, 0, 0   # 未使用的参数
# )
# vehicle.send_mavlink(msg)
# print("bomb command 2 sent")

vehicle.close()

print("Completed")
