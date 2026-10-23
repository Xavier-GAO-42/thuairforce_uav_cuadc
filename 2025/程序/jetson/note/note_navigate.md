# 2025 SITL、航点与 DroneKit 开发笔记

## 目录

- [1 Ardupilot-sitl 安装](#section-1)
- [2 Ardupilot-sitl 模拟器使用](#section-2)
- [3 航点文件](#section-3)
- [4 我的代码](#section-4)
- [5 关于dronekit](#section-5)
- [6 以下是dronekit在处理Missions的使用说明文档](#section-6)
  - [6.1 参考文档摘录](#section-7)
  - [6.2 Missions (AUTO Mode)](#section-8)
  - [6.3 Mission Command Overview](#section-9)
  - [6.4 Download current mission](#section-10)
  - [6.5 Clearing current mission](#section-11)
  - [6.6 Creating/adding mission commands](#section-12)
  - [6.7 Modifying missions](#section-13)
  - [6.8 Running and monitoring missions](#section-14)
  - [6.9 Handling the end of a mission](#section-15)
  - [6.10 Useful Mission functions](#section-16)
    - [6.10.1 Load a mission from a file](#section-17)
    - [6.10.2 Save a mission to a file](#section-18)
    - [6.10.3 Get distance to waypoint](#section-19)

<a id="section-1"></a>

## 1 Ardupilot-sitl 安装

1. 克隆ardupilot的仓库
```bash
git clone https://github.com/ArduPilot/ardupilot.git
```
2. 执行脚本安装依赖
**在执行脚本之前，最好做一个python虚拟环境，如果你不想这些包被安装在全局的话**
```
cd ardupilot
Tools/environment_install/install-prereqs-ubunut.sh -y # 根据自己的平台选择对应的脚本，都在这个目录下
```
3. 编译固件
```
cd ArduPlane # 选择想要编译的固件类型，这个是固定翼，ArduCopter是四旋翼，还有其他的
../Tools/autotest/sim_vehicle.py -w
```
大约需要编译1300多个.cpp文件。编译结束后即可退出
4. 运行sitl
```
../Tools/autotest/sim_vehicle.py --map --console
```
这时在命令行窗口会出现一个终端，这个终端是mavproxy
```
MANUAL>
```
输入help查看支持的指令。常用的有wp、output、arm、mode等
```
modo auto # 切换到AUTO模式
arm throttle # 解锁引擎
output list # 查看mavlink输出端口
```
项目整体文档：https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
mavproxy文档见https://ardupilot.org/mavproxy、https://ardupilot.org/dev/docs/plane-sitlmavproxy-tutorial.html
安装流程参考https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html

<a id="section-2"></a>

## 2 Ardupilot-sitl 模拟器使用

- 首先启动``conda activate ardupilot``虚拟环境
- 然后 ``cd ardupilot/Arduplane/``
- 启动
```bash
# 德清莫干山机场：
../Tools/autotest/sim_vehicle.py --console --map --custom-location=30.504305555555554,120.10720277777777,4.3,270 # 在跑道正中心向西侧起飞
```
```bash
# 氢能翱翔基地：
../Tools/autotest/sim_vehicle.py --console --map --custom-location=39.5873022,116.1998743,35,190 # 从跑道由北向南起飞
../Tools/autotest/sim_vehicle.py --console --map --custom-location=39.58669139,116.20417966,35,270 # 在东侧向西侧起飞
```
在mavproxy终端中执行
```
wp load <WP_FILE_PATH> # 也通过在map窗口右键-Mission-Load从图形界面选择航点文件 航点文件具体说明见后
wp list # 看一下上传的航点是否正确
arm throttle # 解锁引擎
mode auto # 切换到auto模式
# 此时飞机就应当起飞了。模拟器中是滑跑起飞，现实中一般是抛飞。
```

<a id="section-3"></a>

## 3 航点文件

该航点文件可用mission-planner导出，或者用该项目的mission.py生成。QGC WPL 110格式即可。
需要注意的是，第0航点用于表示home location，实际上0号航点是由飞控根据解锁时的位置自动生成的。1号航点是takeoff时才能起飞。dronekit只需要上传1号航点takeoff开始的missionlist(即第一个航点就是起飞，cmds[0]为takeoff)。在目前代码中，由mission.py读取的航点文件是在mission-planner导出航点文件的基础上删除0号航点(即第二行)后得到的。
```waypoint```文件夹下有一些示例的航点文件可用于直接用于sitl测试(包含0号航点)。文件名的qingneng表示这些航点是在氢能翱翔基地附近的。
从而言之，dronekit上传的航点是从1号航点takeoff开始的，0号航点由飞控在解锁时自动生成。而mission_planner导出的和sitl mavproxy终端读取的航点都是要包含0号航点的。

<a id="section-4"></a>

## 4 我的代码

使用dronekit来与飞控交互的程序主要是navigate.py，这个程序还用到了route_plan.py（航线规划程序）和mission.py（提供一些给飞控上传、下载航点与读取文件中的mission并转化为cmd_list的实用函数）

我写了一个main_test_navigate.py专门用来测试navigate。

可以用它输出的日志（保存在/log/navigate_test文件夹下），以及这个文件夹下navigate_analyzer.py的小程序分析侦查航线的执行情况（-c 指定日志路径）

运行main_test_navigate.py之前记得确认里面的配置设置
```python
config_file_path="./config/config_sitl_30m.json"
```
在运行模拟器的时候，配置中要通过网络连接模拟飞控
```json
{
  "navigate": {
    "connection_string": "udpin:127.0.0.1:14550",
    ...
  }
}
```
而连接真飞控，要通过/tty/ACM0串口连接 后面还要写波特率
```json
{
  "navigate": {
    "connection_string": "/dev/ttyACM0,57600",
    ...
  }
}
```

<a id="section-5"></a>

## 5 关于dronekit

https://dronekit-python.readthedocs.io/en/latest/about/index.html
dronekit本质上都是基于mavlink实现的（包括获取信息，发送指令）mavlink就是和飞控通讯的协议
https://mavlink.io/

主要就是用来上传航点指令，请求与获取mavlink数据包。别的也没什么了。
关于automode与mission的介绍：https://dronekit-python.readthedocs.io/en/latest/guide/auto_mode.html


ardupilot关于mission command的文档
https://ardupilot.org/plane/docs/common-mission-planning.html
https://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html

ardupilot选用了MAV_CMD中一些适用于无人机的指令，对于其他不支持的指令，ardupilot在收到后会直接丢弃

MP地面站导出的指令格式长这样：
```
index   wp      frame   cmd_id  p1  p2  p3  p4  p5          p6              p7  autocontinue
2       0	    3	    16      0	0	0	0	39.58558310	116.19948730	30	1
```
- index:      指令序号，但在程序中**不需要**
- currentwp:  不用管，不需要，**都设为0**
            
这个的意思是上传一串指令，飞机从currentwp=1的航点开始执行任务。执行完毕后这个任务的currentwp设为0，下一个任务的currentwp设为1

- frame:      MAV_FRAME变量，参见https://mavlink.io/en/messages/common.html#MAV_FRAME

Ardupilot中规定：
All other commands use the MAV_FRAME_GLOBAL_RELATIVE_ALT frame, which uses the same latitude and longitude, but sets altitude as relative to the home position (home altitude = 0).
指令统一使用**WGS84地心坐标系**，经纬度为**度分格式(不是度分秒)**，高度为**相对家的高度**(frame=3,RELATIVE_ALT)

Plane commands can additionally use MAV_FRAME_GLOBAL_TERRAIN_ALT frame of reference. This again has the same WGS84 frame of reference for latitude/longitude, but specifies altitude relative to ground height (as defined in a terrain database).

也可以换用另一种高度表达方式，高度为相对当前位置地面的高度(frame=10,TERRAIN_ALT)但我们用不到，因为场地是平地

- command:    指令ID, 根据MAV_CMD中的id规定。对照表见https://mavlink.io/en/messages/common.html#MAV_CMD
- param1~param7: 指令所需要的参数

参数定义见文档ardupilot
https://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html
(**但是文档中也有错误的地方，missionplanner飞行计划中下面表格有显示参数名称，这个更准确。多跑模拟器，多尝试**)譬如loiter_to_alt，实际上param2才是盘旋半径，但文档中说第一个是盘旋半径

- autocontinue:   =1的时候，若这一指令执行完毕，飞机自动执行序列中的下一个命令.**都设置为1**

readmission函数读取这一个格式的文件，将每行转化成dronekit提供的指令对象：
```
cmd = Command(
    0,                # target_system (目标系统ID)
    0,                # target_component (目标组件ID)
    0,                # seq (序列号，通常设为0，上传时自动分配)。
                      # 以上三个参数都不用管，因为我们只需要控制一架飞机
    frame,         # 坐标系 (如 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT = 3)，也由mavlink规定
    command,       # mavlink命令ID (如 16=航点, 22=起飞)
    currentwp,     # 是否是当前正在执行的航点 (0/1) 
    autocontinue,  # 自动继续标志 (0或1)
    param1,        # 参数1，mavlink指令规定的参数1~7
    param2,        # 参数2
    param3,        # 参数3
    param4,        # 参数4
    param5,        # 参数5 (通常是纬度)
    param6,        # 参数6 (通常是经度)
    param7         # 参数7 (通常是高度)
)
```
修改飞控航点：获取航点、编辑航点、上传航点并自动执行
```
cmds = vehicle.commands # (从飞控下载)获取目前正在执行的航点
cmds.clear()
for command in missionlist:
    cmds.add(command)
vehicle.commands.upload()
```


<a id="section-6"></a>

## 6 以下是dronekit在处理Missions的使用说明文档

<a id="section-7"></a>

### 6.1 参考文档摘录

<a id="section-8"></a>

### 6.2 Missions (AUTO Mode)

AUTO mode is used run pre-defined waypoint missions on Copter, Plane and Rover.

DroneKit-Python provides basic methods to download and clear the current mission commands from the vehicle, to add and upload new mission commands, to count the number of waypoints, and to read and set the currently executed mission command. You can build upon these basic primitives to create high-level mission planning functionality.

This section shows how to use the basic methods and provides a few useful helper functions. Most of the code can be observed running in Example: Basic Mission and Example: Mission Import/Export.

Note

We recommend that you use GUIDED mode instead of AUTO mode where possible, because it offers finer and more responsive control over movement, and can emulate most mission planning activities.

AUTO mode can be helpful if a command you need is not supported in GUIDED mode on a particular vehicle type.
<a id="section-9"></a>

### 6.3 Mission Command Overview

The mission commands (e.g. MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_WAYPOINT ) supported for each vehicle type are listed here: Copter, Plane, Rover.

There are three types of commands:

    NAVigation commands (MAV_CMD_NAV_*) are used to control vehicle movement, including takeoff, moving to and around waypoints, changing altitude, and landing.

    DO commands (MAV_CMD_DO_*) are for auxiliary functions that do not affect the vehicle’s position (for example, setting the camera trigger distance, or setting a servo value).

    CONDITION commands (MAV_CMD_NAV_*) are used to delay DO commands until some condition is met. For example MAV_CMD_CONDITION_DISTANCE will prevent DO commands executing until the vehicle reaches the specified distance from the waypoint.

During a mission at most one NAV command and one DO or CONDITION command can be running at the same time. CONDITION and DO commands are associated with the last NAV command that was sent: if the UAV reaches the waypoint before these commands are executed, the next NAV command is loaded and they will be skipped.

The MAVLink Mission Command Messages (MAV_CMD) wiki topic provides a more detailed overview of commands.

Note

    If the autopilot receives a command that it cannot handle, then the command will be (silently) dropped.

    You cannot yet determine dynamically what commands are supported. We hope to deliver this functionality in the forthcoming capability API.

<a id="section-10"></a>

### 6.4 Download current mission

The mission commands for a vehicle are accessed using the Vehicle.commands attribute. The attribute is of type CommandSequence, a class that provides ‘array style’ indexed access to the waypoints which make up the mission.

Waypoints are not downloaded from vehicle until download() is called. The download is asynchronous; use wait_ready() to block your thread until the download is complete:

```python
# Connect to the Vehicle (in this case a simulated vehicle at 127.0.0.1:14550)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Download the vehicle waypoints (commands). Wait until download is complete.
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
```

Note

In DroneKit-Python 2 Vehicle.commands contains just the editable waypoints (in version 1.x, the commands included the non-editable home location as the first item).
<a id="section-11"></a>

### 6.5 Clearing current mission

To clear a mission you call clear() and then Vehicle.commands.upload() (to upload the changes to the vehicle):

```python
# Connect to the Vehicle (in this case a simulated vehicle at 127.0.0.1:14550)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Get commands object from Vehicle.
cmds = vehicle.commands

# Call clear() on Vehicle.commands and upload the command to the vehicle.
cmds.clear()
cmds.upload()
```

Note

If a mission that is underway is cleared, the mission will continue to the next waypoint. If you don’t add a new command before the waypoint is reached then the vehicle mode will change to RTL (return to launch) mode.
<a id="section-12"></a>

### 6.6 Creating/adding mission commands

After downloading or clearing a mission new commands can be added and uploaded to the vehicle. Commands are added to the mission using add() and are sent to the vehicle (either individually or in batches) using upload().

Each command is packaged in a Command object (see that class for the order/meaning of the parameters). The supported commands for each vehicle are linked above.

```python
# Connect to the Vehicle (in this case a simulated vehicle at 127.0.0.1:14550)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Get the set of commands from the vehicle
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

# Create and add commands
cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10)
cmd2=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 10, 10, 10)
cmds.add(cmd1)
cmds.add(cmd2)
cmds.upload() # Send commands
```

<a id="section-13"></a>

### 6.7 Modifying missions

While you can add new commands after downloading a mission it is not possible to directly modify and upload existing commands in Vehicle.commands (you can modify the commands but changes do not propagate to the vehicle).

Instead you copy all the commands into another container (e.g. a list), modify them as needed, then clear Vehicle.commands and upload the list as a new mission:

```python
# Connect to the Vehicle (in this case a simulated vehicle at 127.0.0.1:14550)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Get the set of commands from the vehicle
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

# Save the vehicle commands to a list
missionlist=[]
for cmd in cmds:
    missionlist.append(cmd)

# Modify the mission as needed. For example, here we change the
# first waypoint into a MAV_CMD_NAV_TAKEOFF command.
missionlist[0].command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF

# Clear the current mission (command is sent when we call upload())
cmds.clear()

#Write the modified mission and flush to the vehicle
for cmd in missionlist:
    cmds.add(cmd)
cmds.upload()
```

The changes are not guaranteed to be complete until upload() is called on the parent Vehicle.commands object.
<a id="section-14"></a>

### 6.8 Running and monitoring missions

To start a mission, change the mode to AUTO:

```python
# Connect to the Vehicle (in this case a simulated vehicle at 127.0.0.1:14550)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Set the vehicle into auto mode
vehicle.mode = VehicleMode("AUTO")
```

Note

If the vehicle is in the air, then changing the mode to AUTO is all that is required to start the mission.

Copter 3.3 release and later: If the vehicle is on the ground (only), you will additionally need to send the MAV_CMD_MISSION_START command.

You can stop/pause the current mission by switching out of AUTO mode (e.g. into GUIDED mode). If you switch back to AUTO mode the mission will either restart at the beginning or resume at the current waypoint - the behaviour depends on the value of the MIS_RESTART parameter (available on all vehicle types).

You can monitor the progress of the mission by polling the Vehicle.commands.next attribute to get the current command number. You can also change the current command by setting the attribute to the desired command number.

```python
vehicle.commands.next=2
print "Current Waypoint: %s" % vehicle.commands.next
vehicle.commands.next=4
print "Current Waypoint: %s" % vehicle.commands.next
```

There is no need to upload() changes to send an update to the next attribute to the vehicle (and as with other attributes, if you fetch a value, it is updated from the vehicle).
<a id="section-15"></a>

### 6.9 Handling the end of a mission

At the end of the mission the vehicle will enter LOITER mode (hover in place for Copter, circle for Plane, stop for Rover). You can add new commands to the mission, but you will need to toggle from/back to AUTO mode to start it running again.

Currently there is no notification in DroneKit when a mission completes. If you need to detect mission end (in order to perform some other operation) then you can either:

    Add a dummy mission command and poll Vehicle.commands.next for the transition to the final command, or

    Compare the current position to the target position in the final waypoint.

<a id="section-16"></a>

### 6.10 Useful Mission functions

This example code contains a number of functions that might be useful for managing and monitoring missions:
<a id="section-17"></a>

#### 6.10.1 Load a mission from a file

upload_mission() uploads a mission from a file.

The implementation calls readmission() (below) to import the mission from a file into a list. The method then clears the existing mission and uploads the new version.

Adding mission commands is discussed here in the guide.

```python
def upload_mission(aFileName):
        """
        Upload a mission from a file.
        """
        #Read mission from file
        missionlist = readmission(aFileName)

        print "\nUpload mission from a file: %s" % import_mission_filename
        #Clear existing mission from vehicle
        print ' Clear mission'
        cmds = vehicle.commands
        cmds.clear()
        #Add new mission to vehicle
        for command in missionlist:
            cmds.add(command)
        print ' Upload mission'
        vehicle.commands.upload()
```

readmission() reads a mission from the specified file and returns a list of Command objects.

Each line is split up. The first line is used to test whether the file has the correct (stated) format. For subsequent lines the values are stored in a Command object (the values are first cast to the correct float and int types for their associated parameters). The commands are added to a list which is returned by the function.

```python
def readmission(aFileName):
    """
    Load a mission from a file into a list.

    This function is used by upload_mission().
    """
    print "Reading mission from file: %s\n" % aFileName
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist
```

<a id="section-18"></a>

#### 6.10.2 Save a mission to a file

save_mission() saves the current mission to a file (in the Waypoint file format). It uses download_mission() (below) to get them mission, and then writes the list line-by-line to the file.

```python
def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    missionlist = download_mission()
    output='QGC WPL 110\n'
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        file_.write(output)
```

download_mission() downloads the Vehicle.commands from the vehicle and adds them to a list. Downloading mission is discussed in the guide.

```python
def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist
```

<a id="section-19"></a>

#### 6.10.3 Get distance to waypoint

distance_to_current_waypoint() returns the distance (in metres) to the next waypoint:

```python
def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint=vehicle.commands.next
    if nextwaypoint ==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat=missionitem.x
    lon=missionitem.y
    alt=missionitem.z
    targetWaypointLocation=LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint
```

The function determines the current target waypoint number with Vehicle.commands.next and uses it to index the commands to get the latitude, longitude and altitude of the target waypoint. The get_distance_metres() function (see Frame conversion functions) is then used to calculate and return the (horizontal) distance from the current vehicle location.

Tip

This implementation is very basic. It assumes that the next command number is for a valid NAV command (it might not be) and that the lat/lon/alt values are non-zero. It is however a useful indicator for test code.
