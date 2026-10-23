# 自动挂载或卸载nfs
import subprocess
import os
import time
import logging

logger = logging.getLogger("NFS mount")

def mount_nfs(nfs_server, nfs_share, mount_point, options=None):
    """
    自动挂载NFS共享
    
    参数:
    nfs_server: NFS服务器IP或主机名
    nfs_share: 共享目录路径
    mount_point: 本地挂载点
    options: 挂载选项（可选）
    """
    
    # 检查挂载点目录是否存在，不存在则创建
    if not os.path.exists(mount_point):
        os.makedirs(mount_point)
        logger.info(f"create local mount point: {mount_point}")
    
    # 检查是否已经挂载
    if os.path.ismount(mount_point):
        logger.info(f"{mount_point} has mounted")
        return True
    
    # 构建挂载命令
    nfs_path = f"{nfs_server}:{nfs_share}"
    mount_cmd = ["sudo", "mount", "-t", "nfs"]
    
    if options:
        mount_cmd.extend(["-o", options])
    
    mount_cmd.extend([nfs_path, mount_point])
    
    try:
        # 执行挂载命令
        result = subprocess.run(mount_cmd, check=True, capture_output=True, text=True,timeout=5)
        logger.info(f"NFS挂载成功: {nfs_path} -> {mount_point}")
        return True
        
    except subprocess.CalledProcessError as e:
        logger.info(f"挂载失败: {e}")
        logger.info(f"错误输出: {e.stderr}")
        return False

def unmount_nfs(mount_point):
    """卸载NFS共享"""
    try:
        if os.path.ismount(mount_point):
            result = subprocess.run(["sudo", "umount", mount_point], check=True)
            logger.info(f"卸载成功: {mount_point}")
            return True
        else:
            logger.info(f"{mount_point} 未挂载")
            return True
    except subprocess.CalledProcessError as e:
        logger.info(f"卸载失败: {e}")
        return False

# 使用示例
if __name__ == "__main__":
    # NFS配置
    NFS_SERVER = "192.168.144.25"  # 替换为你的NFS服务器IP
    NFS_SHARE = "/home/thuair/CUADC/picture_nfs"     # 替换为NFS共享路径
    MOUNT_POINT = "./picture_nfs"      # 本地挂载点
    MOUNT_OPTIONS = "defaults"  # 挂载选项
    
    # 执行挂载
    # if mount_nfs(NFS_SERVER, NFS_SHARE, MOUNT_POINT, MOUNT_OPTIONS):
    #     print("NFS挂载完成！")
        
    #     # 这里可以添加你的业务逻辑
    #     # 例如列出挂载目录内容
    #     try:
    #         files = os.listdir(MOUNT_POINT)
    #         print(f"共享目录内容: {files}")
    #     except Exception as e:
    #         print(f"读取目录失败: {e}")

    # 执行卸载
    unmount_nfs(MOUNT_POINT)
