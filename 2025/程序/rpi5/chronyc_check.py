import subprocess
import time
import logging
import re

logger = logging.getLogger('chrony check')
def is_time_highly_synchronized():
    """更严格的时间同步检测（用于高精度应用）"""
    try:
        result = subprocess.run(
            ['chronyc', 'tracking'],
            capture_output=True,
            text=True,
            check=True
        )
        output = result.stdout
        
        # 检查所有关键指标
        checks = {
            'ref_id': False,
            'stratum': False,
            'leap': False,
            'offset': False,
            'sources': False
        }
        
        # 1. Reference ID检查
        ref_id_match = re.search(r'Reference ID\s*:\s*(\S+)', output)
        if ref_id_match:
            ref_id = ref_id_match.group(1)
            if ref_id not in ['7F7F0101', '()', '']:
                checks['ref_id'] = True
                logger.info(f"chrony check: Reference ID is valid: {ref_id}")
        
        # 2. Stratum检查（要求更严格）
        stratum_match = re.search(r'Stratum\s*:\s*(\d+)', output)
        if stratum_match:
            stratum = int(stratum_match.group(1))
            if stratum <= 4:  # 只接受高质量的时间源
                checks['stratum'] = True
                logger.info(f"chrony check: Stratum is great: {stratum}")
        
        # 3. Leap状态检查
        if 'Leap status     : Normal' in output:
            checks['leap'] = True
            logger.info("chrony check: Leap is normal")
        
        # 4. 时间偏移检查（更严格的标准）
        offset_match = re.search(r'System time\s*:\s*([\d.-]+)\s*seconds', output)
        if offset_match and offset_match.group(1) != "0.000000000":
            offset = abs(float(offset_match.group(1)))
            if offset < 0.01:  # 10毫秒以内
                checks['offset'] = True
                logger.info(f"chrony check: time offset is {offset:.6f}s")
        
        # 5. 检查时间源状态
        sources_result = subprocess.run(
            ['chronyc', 'sources'],
            capture_output=True, text=True, check=True
        )
        if '*' in sources_result.stdout:  # 有优选的时间源
            checks['sources'] = True
            logger.info(f"chrony check: sources selected")
        
        # 判断是否高度同步（所有条件都满足）
        highly_synced = all(checks.values())
        
        if highly_synced:
            logger.info("chrony check done")
        else:
            logger.info("chrony check hasn't been done")
            for check, passed in checks.items():
                status = "yes" if passed else "no"
                logger.info(f"  {check}: {status}")
        
        return highly_synced
        
    except Exception as e:
        logger.info(f"chrony check error: {e}")
        return False

def is_time_synchronized():
    """检查系统时间是否已完成同步"""
    try:
        # 获取chronyc跟踪状态
        result = subprocess.run(
            ['chronyc', 'tracking'],
            capture_output=True,
            text=True,
            check=True
        )
        output = result.stdout
        
        # 检查关键指标
        if "Leap status     : Normal" in output:
            # 检查时间偏差（通常小于100ms表示已同步）
            for line in output.splitlines():
                if "System time" in line:
                    time_str = line.split(':')[1].split()[0]
                    logger.info(f"chronyc time offset: {time_str}")
                    try:
                        time_offset = abs(float(time_str))
                        return time_offset < 0.1  # 100ms阈值
                    except ValueError:
                        continue
        return False
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False

def chronyc_checker():    
    logger.info("waiting for time chrony...")
    
    while True:
        if is_time_highly_synchronized():
            logger.info("time synchronized success")
            break
        time.sleep(2)  # 每5秒检查一次