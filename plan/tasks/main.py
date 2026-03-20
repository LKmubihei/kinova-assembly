"""
power_panel 安装任务 - 主入口

流程：
  1. HOME 点 → 采集图像 → 检测背板安装状态（step2）、螺丝孔（step3）
  2. 移动到 BIN_SPECT → 采集图像 → 检测料箱中是否有 power_panel（step1）
  3. HTN 规划
用法：
    python3 plan/tasks/main.py
    python3 plan/tasks/main.py --mock
"""

import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from hyy_message.action import MoveXYZW

# 路径设置
_PLAN_ROOT = Path(__file__).resolve().parent.parent.parent
_HTN_ROOT  = Path(__file__).resolve().parent.parent / "htn"
for _p in [str(_PLAN_ROOT), str(_HTN_ROOT)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from robot_htn import htn, State
from robot_htn.helpers import print_state

import plan.tasks.operators as _ops   # noqa: F401
import plan.tasks.methods  as _mth   # noqa: F401

# ---------------------------------------------------------------------------
# 点位定义
# ---------------------------------------------------------------------------
HOME      = (0.30, -0.38, 0.15, -math.pi, 0.0, 0.0)
BIN_SPECT = (-0.40, 0.25, 0.25, -math.pi, 0.0, 0.0)
SPEED, ACCEL = 0.20, 0.10


# ---------------------------------------------------------------------------
# 机械臂客户端
# ---------------------------------------------------------------------------
class ArmClient(Node):
    def __init__(self):
        super().__init__("power_panel_task_node")
        self._ac = ActionClient(self, MoveXYZW, "MoveXYZW")
        self.get_logger().info("等待 MoveXYZW action server …")
        self._ac.wait_for_server()
        self.get_logger().info("Action server 已就绪")

    def move(self, pose: tuple, label: str = "") -> bool:
        x, y, z, roll, pitch, yaw = pose
        goal = MoveXYZW.Goal()
        goal.positionx = x
        goal.positiony = y
        goal.positionz = z
        goal.roll  = roll
        goal.pitch = pitch
        goal.yaw   = yaw
        goal.speed = max(min(SPEED, 0.2), 0.001)
        goal.accel = max(min(ACCEL, 0.2), 0.001)

        self.get_logger().info(f"[{label}] → ({x:.3f}, {y:.3f}, {z:.3f})")
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


# ---------------------------------------------------------------------------
# 相机采集
# ---------------------------------------------------------------------------
def capture_frame() -> np.ndarray:
    """启动 D415，预热后采集一帧，返回 BGR 图像。"""
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    profile  = pipeline.start(config)

    try:
        device       = profile.get_device()
        color_sensor = next(
            s for s in device.query_sensors()
            if s.get_info(rs.camera_info.name) == "RGB Camera"
        )

        if color_sensor.supports(rs.option.enable_auto_exposure):
            color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        if color_sensor.supports(rs.option.enable_auto_white_balance):
            color_sensor.set_option(rs.option.enable_auto_white_balance, 1)

        print("[相机] 开始预热 3 秒，等待自动曝光/白平衡稳定 …")
        t0 = time.time()
        color_frame = None
        while time.time() - t0 < 3.0:
            frames      = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

        if not color_frame:
            raise RuntimeError("未能从 D415 获取彩色帧")

        for _ in range(10):
            frames      = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

        image_rgb = np.asanyarray(color_frame.get_data())
        return cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

    finally:
        pipeline.stop()


# ---------------------------------------------------------------------------
# 感知
# ---------------------------------------------------------------------------
def perceive_bin(image: np.ndarray, save_dir=None) -> bool:
    """step1：料箱中是否有 power_panel。"""
    from plan.perception.bin_detector import BinDetector
    tmp = Path("/tmp/task_step1.png")
    cv2.imwrite(str(tmp), image)
    res = BinDetector().detect(tmp, save_dir=save_dir)
    print(f"  [料箱检测] found={res['found']}  count={res['count']}")
    return res["found"]


def perceive_panel(image: np.ndarray, save_dir=None) -> bool:
    """step2：power_panel 是否安装到位。"""
    from plan.perception.panel_detector import PanelDetector
    tmp = Path("/tmp/task_step2.png")
    cv2.imwrite(str(tmp), image)
    res = PanelDetector().detect(tmp, save_dir=save_dir)
    print(f"  [安装验证] installed={res['installed']}  count={res['count']}")
    return res["installed"]


def perceive_screws(image: np.ndarray, save_dir=None) -> dict:
    """step3：螺丝孔位置检测（图像旋转180度后检测）。"""
    from plan.perception.screw_detector import ScrewDetector
    rotated = cv2.rotate(image, cv2.ROTATE_180)
    tmp = Path("/tmp/task_step3.png")
    cv2.imwrite(str(tmp), rotated)
    res   = ScrewDetector().detect(tmp, save_dir=save_dir)
    holes = {k: v["position"] for k, v in res["holes"].items() if v["position"]}
    print(f"  [螺丝孔检测] detected={list(holes.keys())}")
    return holes


# ---------------------------------------------------------------------------
# HTN 状态 & 规划
# ---------------------------------------------------------------------------
def make_state(bin_result: bool, panel_result: bool, screw_result: dict) -> State:
    s = State("power_panel_install")
    s.robot_at                 = "home"
    s._perception_bin_result   = bin_result
    s.bin_inspected            = False
    s.bin_has_panel            = False
    s.holding_panel            = False
    s._perception_panel_result = panel_result
    s.panel_placed             = False
    s.placement_verified       = False
    s.placement_ok             = False
    s._perception_screw_result = screw_result
    s.holes_detected           = False
    s.hole_positions           = {}
    s.holding_screw            = False
    s.screw_aligned_to         = None
    s.screw_inserted_at        = None
    s.screws_available         = 10
    s.screws_fastened          = []
    return s


def print_plan(plan):
    print("\n" + "=" * 56)
    print("HTN 规划结果")
    print("=" * 56)
    if not plan:
        print("规划失败：无法找到可行计划")
        return
    print(f"共 {len(plan)} 步\n")
    for i, action in enumerate(plan, 1):
        name    = action[0]
        args    = action[1:] if len(action) > 1 else ()
        args_str = f"({', '.join(str(a) for a in args)})" if args else "()"
        print(f"  {i:3d}. {name}{args_str}")
    print("=" * 56)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="power_panel 安装任务")
    parser.add_argument("--mock", action="store_true", help="跳过相机和机械臂，使用模拟结果")
    parser.add_argument("--verbose", type=int, default=0)
    args = parser.parse_args()

    print("=" * 56)
    print("power_panel 安装 — HTN 规划系统")
    print("=" * 56)

    # 带时间戳的结果保存目录
    from datetime import datetime
    _RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"
    save_dir = _RESULTS_DIR / datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir.mkdir(parents=True, exist_ok=True)
    print(f"结果保存目录: {save_dir}")

    if args.mock:
        print("[mock] 使用模拟感知结果")
        bin_result   = True
        panel_result = True
        screw_holes  = {f"hole_{c}": (i * 100.0, 200.0) for i, c in enumerate("ABCDEFG")}
    else:
        rclpy.init()
        arm = ArmClient()

        try:
            # --- 阶段1：BIN_SPECT 点采集（料箱检测）---
            # print("\n[阶段 1] 移动到 BIN_SPECT 点 …")
            # if not arm.move(BIN_SPECT, "BIN_SPECT"):
            #     arm.get_logger().error("移动到 BIN_SPECT 失败，退出")
            #     return

            # print("[阶段 1] 采集图像 …")
            # image_bin = capture_frame()
            # cv2.imwrite(str(save_dir / "step1_raw.png"), image_bin)

            # print("[阶段 1] 感知：料箱检测")
            # bin_result = perceive_bin(image_bin, save_dir=save_dir / "step1")

            # --- 阶段2：返回 HOME 采集（背板安装验证 + 螺丝孔）---
            print("\n[阶段 2] 返回 HOME 点 …")
            if not arm.move(HOME, "HOME"):
                arm.get_logger().error("移动到 HOME 失败，退出")
                return

            print("[阶段 2] 采集图像 …")
            image_home = capture_frame()
            cv2.imwrite(str(save_dir / "step2_raw.png"), image_home)
            cv2.imwrite(str(save_dir / "step3_raw.png"), image_home)

            print("[阶段 2] 感知：安装验证 + 螺丝孔")
            panel_result = perceive_panel(image_home, save_dir=save_dir / "step2")
            screw_holes  = perceive_screws(image_home, save_dir=save_dir / "step3")

        finally:
            arm.destroy_node()
            rclpy.shutdown()

    # --- HTN 规划 ---
    if not bin_result:
        print("\n[中止] 料箱中未检测到 power_panel，无法继续规划。")
        sys.exit(1)

    state = make_state(bin_result, panel_result, screw_holes)
    print("\n初始状态:")
    print_state(state)

    plan = htn.plan(
        state,
        [("install_power_panel",)],
        htn.get_operators(),
        htn.get_methods(),
        verbose=args.verbose,
    )
    print_plan(plan)
    return plan


if __name__ == "__main__":
    main()
