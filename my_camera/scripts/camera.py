import v4l2
import numpy as np
import fcntl
import os
import mmap
import cv2

# 打开设备
video_device = '/dev/video0'
video_fd = os.open(video_device, os.O_RDWR)

# 设置 V4L2 格式
fmt = v4l2.v4l2_format()
fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
fmt.fmt.pix.width = 640
fmt.fmt.pix.height = 480
fmt.fmt.pix.pixelformat = v4l2.V4L2_PIX_FMT_YUYV
fcntl.ioctl(video_fd, v4l2.VIDIOC_S_FMT, fmt)

# 获取帧
buffer = mmap.mmap(video_fd, 640*480*2, mmap.MAP_SHARED, mmap.PROT_READ)

# 获取并显示图像
while True:
    frame = np.frombuffer(buffer, dtype=np.uint8).reshape((480, 640, 2))
    frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)

    cv2.imshow('Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
