# 主要提供跟上传、下载、清除飞控任务的一些工具函数
# 以及从文件中读取航点、将航点写入文件的工具函数
# 与dronekit有关的函数(即要用到vehicle对象的函数)只在navigate.py中被调用
import logging
from dronekit import Command

"""logging.basicConfig(
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    level=logging.INFO
)"""
logger = logging.getLogger(__name__)

def readmission(aFileName):
    """
    Load a mission from a file into a list.
    返回的是指令列表 每个元素都是dronekit提供的Command类型
    参数: aFileName: 航点文件与当前python工作路径的相对路径
    """
    logger.info("Reading mission from file: %s" % aFileName)

    missionlist = []
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i == 0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray = line.split('\t')
                ln_index = int(linearray[0])
                ln_currentwp = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1 = float(linearray[4])
                ln_param2 = float(linearray[5])
                ln_param3 = float(linearray[6])
                ln_param4 = float(linearray[7])
                ln_param5 = float(linearray[8])
                ln_param6 = float(linearray[9])
                ln_param7 = float(linearray[10])
                ln_autocontinue = int(linearray[11].strip())
                cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2,
                                ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                
                missionlist.append(cmd)
    return missionlist

def clean_mission(vehicle):
    """用于在起飞之前严格清理飞控残留航点"""
    cmds = vehicle.commands
    cmds.download()
    if not cmds.wait_ready(timeout=60):
        logger.error("clean: Timeout waiting for initial commands download!")
        return False
    if cmds.count == 0:
        logger.info("clean: Mission is already empty")
        return True
    cmds.clear()
    if not cmds.wait_ready(timeout=60):
        logger.error("clean: Timeout waiting for command clear!")
        return False
        # 处理错误
    if cmds.count > 0:
        return False
    upload_success = False
    for retry in range(3):  # 最多重试3次
        cmds.upload()
        if cmds.wait_ready(timeout=60):  # 等待上传确认
            logger.info(f'clean: Upload succeeded after {retry+1} attempt(s)')
            upload_success = True
            break
        logger.warning(f"clean: Upload timeout, retrying ({retry+1}/3)...")
    if not upload_success:
        logger.warning(f"clean: Upload Failed")
        return False
    else:
        cmds.download()
        if cmds.wait_ready(timeout=60) and cmds.count == 0:
            logger.info("Clean Mission successful!")
            return True
        else:
            logger.warning("Post-upload verification failed, but mission may still be valid")
            return False

def upload_mission_file(vehicle,filename):
    logger.info(f"upload mission from file {filename}")
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready(timeout=60)
    cmds.clear()
    missionlist=readmission(filename)
    for command in missionlist:
        cmds.add(command)
    #logger.info("print the cmds goint to upload")
    #for cmd in cmds:
    #    print(cmd)
    cmds.upload() # Send commands
    vehicle.commands.next = 0
    cmds.download()
    cmds.wait_ready(time_out=60)

def upload_mission_list(vehicle,missionlist):
    logger.info("upload mission list")
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready(timeout=60)
    cmds.clear()
    for command in missionlist:
        cmds.add(command)
    #logger.info("print the cmds goint to upload")
    #for cmd in cmds:
    #    print(cmd)
    cmds.upload() # Send commands
    cmds.download()
    cmds.wait_ready(timeout=60)
    vehicle.commands.next = 0 # IMPORTANT!!!

def read_waypoints_from_file(filename):
    waypoints = []
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    # Skip the file header
    for line in lines:
        line = line.strip()
        if not line or line.startswith('QGC WPL'):
            continue
        
        # Split fields, supporting tabs or spaces
        fields = line.split('\t')
        
        try:
            # Parse waypoint fields and save as a tuple
            command = int(fields[3])
            latitude = float(fields[8])
            longitude = float(fields[9])
            altitude = float(fields[10])
            
            waypoint = (command, latitude, longitude, altitude)
            waypoints.append(waypoint)
        except ValueError as ve:
            logging.warning('Error {} in line: {}'.format(ve, line))
            continue
    return waypoints

def download_mission(vehicle):
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready(timeout=60)
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

def save_mission_file(aFileName,vehicle):
    """
    Save a mission in the Waypoint file format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    missionlist = download_mission(vehicle)
    output='QGC WPL 110\n'
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        file_.write(output)

def save_mission_list2(aFileName,missionlist):
    """
    Save a mission in the Waypoint file format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    output='QGC WPL 110\n'
    for (i, cmd) in enumerate(missionlist):
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (i+1,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        file_.write(output)

def save_mission_list(aFileName,missionlist):
    """
    Save a mission in the Waypoint file format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    output='QGC WPL 110\n'
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        file_.write(output)
