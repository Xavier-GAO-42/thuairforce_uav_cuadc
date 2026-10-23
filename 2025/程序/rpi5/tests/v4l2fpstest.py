import time
import v4l2
import fcntl
import os

def test_max_fps(device="/dev/video0", duration=5):
    fd = os.open(device, os.O_RDWR)
    
    # 设置格式 (匹配你的实际使用格式)
    fmt = v4l2.v4l2_format()
    fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
    fmt.fmt.pix.width = 1920
    fmt.fmt.pix.height = 1080
    fmt.fmt.pix.pixelformat = v4l2.v4l2_fourcc('M', 'J', 'P', 'G')
    fcntl.ioctl(fd, v4l2.VIDIOC_S_FMT, fmt)
    
    # 请求缓冲
    req = v4l2.v4l2_requestbuffers()
    req.count = 4
    req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
    req.memory = v4l2.V4L2_MEMORY_MMAP
    fcntl.ioctl(fd, v4l2.VIDIOC_REQBUFS, req)
    
    # 开始流
    buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
    fcntl.ioctl(fd, v4l2.VIDIOC_STREAMON, buf_type)
    
    # 连续捕获测试
    count = 0
    start = time.time()
    while time.time() - start < duration:
        buf = v4l2.v4l2_buffer()
        buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        buf.memory = v4l2.V4L2_MEMORY_MMAP
        fcntl.ioctl(fd, v4l2.VIDIOC_DQBUF, buf)
        count += 1
        fcntl.ioctl(fd, v4l2.VIDIOC_QBUF, buf)
    
    fps = count / duration
    print(f"Max FPS: {fps:.1f}")
    
    # 清理
    fcntl.ioctl(fd, v4l2.VIDIOC_STREAMOFF, buf_type)
    os.close(fd)

test_max_fps()