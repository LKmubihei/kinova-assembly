#!/usr/bin/env python3
"""
detect_at_home.py
=================
1. 机器人运动到 HOME 点
2. 从 RealSense D415 采集一帧彩色图像（尽量复现 RealSense Viewer 参数）
3. 用 YOLO (best.pt) 做目标检测
4. 打印每个检测目标的类别、置信度和像素坐标
5. 保存原始图像和带标注图像
"""

import math
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from hyy_message.action import MoveXYZW

# YOLO 权重路径（与 train.py 同目录）
YOLO_DIR = Path(__file__).resolve().parent.parent / "yolo"
BEST_PT = YOLO_DIR / "best.pt"

# HOME 点
HOME = (0.30, -0.38, 0.1, -math.pi, 0.0, 0.0)
SPEED, ACCEL = 0.20, 0.10


# =============================================================================
# 机械臂客户端
# =============================================================================
class ArmClient(Node):
    def __init__(self):
        super().__init__("detect_at_home_node")
        self._ac = ActionClient(self, MoveXYZW, "MoveXYZW")
        self.get_logger().info("等待 MoveXYZW action server …")
        self._ac.wait_for_server()
        self.get_logger().info("Action server 已就绪")

    def move(self, x, y, z, roll, pitch, yaw, speed, accel, label="") -> bool:
        goal = MoveXYZW.Goal()
        goal.positionx = x
        goal.positiony = y
        goal.positionz = z
        goal.roll = roll
        goal.pitch = pitch
        goal.yaw = yaw
        goal.speed = max(min(speed, 0.2), 0.001)
        goal.accel = max(min(accel, 0.2), 0.001)

        self.get_logger().info(
            f"[{label}] → ({x:.4f}, {y:.4f}, {z:.4f})"
            f"  rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})"
        )
        future = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(f"[{label}] Goal 被拒绝")
            return False

        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        ok = res_future.result().result.result == "MoveXYZW:SUCCESS"
        if ok:
            self.get_logger().info(f"[{label}] ✓ 到位")
        else:
            self.get_logger().error(f"[{label}] ✗ 失败")
        return ok


# =============================================================================
# RealSense 工具函数
# =============================================================================
def get_rgb_sensor(device):
    """从设备中找到 RGB Camera sensor。"""
    for sensor in device.query_sensors():
        try:
            name = sensor.get_info(rs.camera_info.name)
            if name == "RGB Camera":
                return sensor
        except Exception:
            pass
    raise RuntimeError("未找到 RGB Camera sensor")


def set_if_supported(sensor, option, value, option_name=""):
    """如果 sensor 支持该 option，则设置它。"""
    if sensor.supports(option):
        sensor.set_option(option, value)
        if option_name:
            print(f"[相机] 设置 {option_name} = {value}")
    else:
        if option_name:
            print(f"[相机] 当前设备不支持 {option_name}，已跳过")


# =============================================================================
# D415 采集单帧（尽量复现 Viewer 参数）
# =============================================================================
def capture_frame() -> np.ndarray:
    """
    启动 D415，按 Viewer 参数配置 RGB，相机预热 3 秒后采集一帧。
    返回 OpenCV 使用的 BGR 图像。
    """
    pipeline = rs.pipeline()
    config = rs.config()

    # Viewer 中是 RGB8，这里先对齐 Viewer
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

    profile = pipeline.start(config)

    try:
        device = profile.get_device()
        color_sensor = get_rgb_sensor(device)

        print("\n" + "=" * 60)
        print("  应用 RealSense Viewer 风格参数")
        print("=" * 60)

        # -----------------------------
        # 1) 先设置固定图像控制项（按你截图）
        # -----------------------------
        set_if_supported(color_sensor, rs.option.brightness, 0, "brightness")
        set_if_supported(color_sensor, rs.option.contrast, 50, "contrast")
        set_if_supported(color_sensor, rs.option.gamma, 300, "gamma")
        set_if_supported(color_sensor, rs.option.hue, 0, "hue")
        set_if_supported(color_sensor, rs.option.saturation, 64, "saturation")
        set_if_supported(color_sensor, rs.option.sharpness, 50, "sharpness")
        set_if_supported(color_sensor, rs.option.backlight_compensation, 0, "backlight_compensation")

        # Power line frequency = Auto
        # 常见映射：0=Disabled, 1=50Hz, 2=60Hz, 3=Auto
        if hasattr(rs.option, "power_line_frequency"):
            set_if_supported(color_sensor, rs.option.power_line_frequency, 3, "power_line_frequency(auto)")

        # Global Time Enabled 在 pyrealsense2 中未必总能通过这个接口设置；
        # 如果支持，就设为 1
        if hasattr(rs.option, "global_time_enabled"):
            set_if_supported(color_sensor, rs.option.global_time_enabled, 1, "global_time_enabled")

        # -----------------------------
        # 2) 自动曝光 / 自动白平衡：按 Viewer 保持开启
        # -----------------------------
        set_if_supported(color_sensor, rs.option.enable_auto_exposure, 1, "enable_auto_exposure")
        set_if_supported(color_sensor, rs.option.enable_auto_white_balance, 1, "enable_auto_white_balance")

        # 下面三个值在 Viewer 里显示的是“当时状态”，AE/AWB 开着时不一定固定：
        # Exposure = 166
        # Gain = 64
        # White Balance = 4600
        #
        # 因为 AE/AWB 开启后，相机会自己调这些值，所以这里只作为参考打印，
        # 不强行锁死，以尽量接近 Viewer 的工作方式。

        # -----------------------------
        # 3) 预热：等待 AE/AWB 收敛
        # -----------------------------
        print("[相机] 开始预热 3 秒，等待自动曝光/白平衡稳定 …")
        t0 = time.time()
        color_frame = None
        while time.time() - t0 < 3.0:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

        if not color_frame:
            raise RuntimeError("未能从 D415 获取彩色帧")

        # 再多取几帧，减少刚切换参数后的波动
        for _ in range(10):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

        # -----------------------------
        # 4) 打印当前实际参数
        # -----------------------------
        print("-" * 60)
        if color_sensor.supports(rs.option.exposure):
            print(f"[相机] 当前 Exposure      = {color_sensor.get_option(rs.option.exposure):.3f}")
        if color_sensor.supports(rs.option.gain):
            print(f"[相机] 当前 Gain          = {color_sensor.get_option(rs.option.gain):.3f}")
        if color_sensor.supports(rs.option.white_balance):
            print(f"[相机] 当前 WhiteBalance  = {color_sensor.get_option(rs.option.white_balance):.3f}")
        print("-" * 60)

        # RealSense 返回 RGB，转成 OpenCV 常用 BGR
        image_rgb = np.asanyarray(color_frame.get_data())
        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        print(
            f"[相机] 采集图像尺寸: {image_bgr.shape}  "
            f"时间戳: {color_frame.get_timestamp():.1f} ms"
        )
        print("=" * 60 + "\n")

        return image_bgr

    finally:
        pipeline.stop()


# =============================================================================
# YOLO 推理并打印结果
# =============================================================================
def yolo_detect(image: np.ndarray):
    """对 BGR numpy 图像做 YOLO 推理，打印检测结果，返回 (detections, annotated_image)。"""
    from ultralytics import YOLO

    if not BEST_PT.exists():
        raise FileNotFoundError(f"未找到 YOLO 权重: {BEST_PT}")

    model = YOLO(str(BEST_PT))
    results = model.predict(source=image, imgsz=1280, verbose=False)

    detections = []
    annotated = image.copy()

    print("\n" + "=" * 55)
    print(f"  YOLO 检测结果  (权重: {BEST_PT.name})")
    print("=" * 55)

    for r in results:
        names = r.names
        boxes = r.boxes

        if boxes is None or len(boxes) == 0:
            print("  未检测到任何目标")
            continue

        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            label = names.get(cls_id, str(cls_id))
            entry = {
                "class": label,
                "confidence": conf,
                "bbox_xyxy": (x1, y1, x2, y2),
                "center_px": (cx, cy),
                "wh_px": (x2 - x1, y2 - y1),
            }
            detections.append(entry)

            print(
                f"  [{label}]  conf={conf:.3f}"
                f"  center=({cx:.1f}, {cy:.1f}) px"
                f"  size=({x2-x1} x {y2-y1}) px"
                f"  xyxy=({x1},{y1},{x2},{y2})"
            )

            # 画框
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 标签背景
            text = f"{label} {conf:.2f}"
            (tw, th), baseline = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            ty = max(y1 - 4, th + baseline)
            cv2.rectangle(
                annotated,
                (x1, ty - th - baseline),
                (x1 + tw, ty),
                (0, 255, 0),
                -1,
            )
            cv2.putText(
                annotated,
                text,
                (x1, ty - baseline),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                2,
                cv2.LINE_AA,
            )

            # 中心点
            cv2.circle(annotated, (int(cx), int(cy)), 4, (0, 0, 255), -1)

    print("=" * 55)
    print(f"  共检测到 {len(detections)} 个目标")
    print("=" * 55 + "\n")
    return detections, annotated


# =============================================================================
# 主流程
# =============================================================================
def main():
    rclpy.init()
    arm = ArmClient()

    try:
        # 1. 运动到 HOME
        x, y, z, roll, pitch, yaw = HOME
        ok = arm.move(x, y, z, roll, pitch, yaw, SPEED, ACCEL, "HOME")
        if not ok:
            arm.get_logger().error("运动到 HOME 失败，退出")
            return

        # 2. 采集图像
        arm.get_logger().info("开始采集 D415 图像（Viewer 风格参数）…")
        image = capture_frame()

        # 3. 保存原始图像，方便和 Viewer 对比
        raw_save_path = Path(__file__).parent / "capture_at_home_raw.png"
        cv2.imwrite(str(raw_save_path), image)
        arm.get_logger().info(f"原始图像已保存: {raw_save_path}")

        # 4. YOLO 推理
        arm.get_logger().info("开始 YOLO 推理 …")
        detections, annotated = yolo_detect(image)

        # 5. 保存带标注框的图像
        anno_save_path = Path(__file__).parent / "capture_at_home_annotated.png"
        cv2.imwrite(str(anno_save_path), annotated)
        arm.get_logger().info(f"标注图像已保存: {anno_save_path}")

        if not detections:
            arm.get_logger().warn("未检测到目标")
        else:
            arm.get_logger().info(f"检测完成，共 {len(detections)} 个目标")

    finally:
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()