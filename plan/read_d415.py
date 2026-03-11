import pyrealsense2 as rs
import numpy as np
import cv2

# 创建 pipeline
pipeline = rs.pipeline()
config = rs.config()

# 只启用彩色流
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# 启动相机
pipeline.start(config)

try:
    while True:
        # 获取一帧
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # 转换为 numpy
        color_image = np.asanyarray(color_frame.get_data())

        # 打印图像信息
        print("Image shape:", color_image.shape)
        print("Timestamp:", color_frame.get_timestamp())

        # 显示图像
        cv2.imshow("RealSense Color", color_image)

        key = cv2.waitKey(1)
        if key == ord('s'):
            cv2.imwrite("color.png", color_image)
            print("Saved color.png")

        if key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()