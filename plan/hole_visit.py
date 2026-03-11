#!/usr/bin/env python3
"""
孔位巡访程序 (Hole Visit)
=========================
根据图纸标注，以 B控 为基准，计算各孔位在 base_link 下的绝对坐标，
依次控制机械臂到达 G/D/E/C/F/A 各孔位。

坐标说明
  臂(base_link) → B 控（单位: cm）:
      x = 57.0 cm → 0.570 m
      y = -58.0 cm → -0.580 m
      z = 2.6 cm → 0.026 m

  B 控 → 各孔（单位: cm，XY 平面内偏移，Z 与 B 等高）:
      G: dx=0.3,  dy=14.3
      D: dx=0.3,  dy=18.8
      E: dx=18.9, dy=0.0
      C: dx=28.9, dy=0.0
      F: dx=18.9, dy=18.8
      A: dx=28.9, dy=18.8

修改说明：
  - 根据手眼标定结果更新 B_X / B_Y / B_Z
  - APPROACH_OFFSET_Z: 孔正上方的接近裕量（m）
  - HOME_POSE: 安全归位点
"""

import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from hyy_message.action import MoveXYZW
from hyy_message.srv import Setangle

# =============================================================================
# 位置参数（根据实际标定修改）
# =============================================================================
_CM = 0.01    # cm → m
_MM = 0.001   # mm → m

# B 控基准点在 base_link 下的绝对坐标（臂→B，单位: cm）
B_X =  42.0 * _CM   # 0.570 m
B_Y = -56.5 * _CM   # -0.580 m
B_Z =   12.6 * _CM   # 0.026 m  孔面高度

APPROACH_OFFSET_Z = 50.0 * _MM   # 孔正上方 50 mm 接近高度

# 安全归位点 (x, y, z, roll, pitch, yaw)
HOME = (0.30, -0.42, 0.12, -math.pi, 0.0, 0.0)

# 速度
SPEED_FAST, ACCEL_FAST = 0.20, 0.10
SPEED_SLOW, ACCEL_SLOW = 0.05, 0.03

# 夹爪朝下姿态
_R, _P, _Y = -math.pi, 0.0, 0.0

# =============================================================================
# 各孔相对 B 控的 XY 偏移（cm → m），Z 与 B 等高
# =============================================================================
HOLE_OFFSETS = {
    "B": ( 0.0 * _CM,   0.0 * _CM),  # 基准点
    "G": ( -0.3 * _CM,  14.3 * _CM),
    "D": ( -0.3 * _CM,  18.8 * _CM),
    "E": (-18.9 * _CM,   0.0 * _CM),
    "C": (-28.9 * _CM,   0.0 * _CM),
    "F": (-18.9 * _CM,  18.8 * _CM),
    "A": (-28.9 * _CM,  18.8 * _CM),
}

VISIT_ORDER = ["B", "A", "C", "D", "G", "F", "E"]
# VISIT_ORDER = ["B"]

# =============================================================================
# 机械臂节点（独立，不依赖 assembly_executor）
# =============================================================================
class ArmClient(Node):
    def __init__(self):
        super().__init__("hole_visit_node")
        self._ac = ActionClient(self, MoveXYZW, "MoveXYZW")
        self.get_logger().info("等待 MoveXYZW action server …")
        self._ac.wait_for_server()
        self.get_logger().info("Action server 已就绪")

        self._gripper = self.create_client(Setangle, "/gripper_cmd_Node/ur_gripper_cmd_server")
        self.get_logger().info("等待夹爪服务 …")
        self._gripper.wait_for_service()
        self.get_logger().info("夹爪服务已就绪")

    def move(self, x, y, z, roll, pitch, yaw, speed, accel, label="") -> bool:
        goal = MoveXYZW.Goal()
        goal.positionx = x
        goal.positiony = y
        goal.positionz = z
        goal.roll  = roll
        goal.pitch = pitch
        goal.yaw   = yaw
        goal.speed = max(min(speed, 0.2), 0.001)
        goal.accel = max(min(accel, 0.2), 0.001)

        self.get_logger().info(
            f"[{label}] → ({x:.4f}, {y:.4f}, {z:.4f})"
            f"  rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})"
            f"  spd={speed:.2f}"
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

    def gripper(self, position: int, force: int = 50, label="夹爪") -> bool:
        """控制夹爪开合。position: 0(全闭) ~ 1000(全开)"""
        req = Setangle.Request()
        req.status  = "gripper_cmd"
        req.hand_id = 1
        req.angle   = [int(force), int(position)]
        self.get_logger().info(f"[{label}] position={position}/1000  force={force}")
        future = self._gripper.call_async(req)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        ok = future.result().angle_accepted
        if not ok:
            self.get_logger().error(f"[{label}] 夹爪指令被拒绝")
        return ok

    def go_home(self, label="归位"):
        x, y, z, r, p, y_ = HOME
        self.move(x, y, z, r, p, y_, SPEED_FAST, ACCEL_FAST, label)


# =============================================================================
# 主流程
# =============================================================================
def visit_holes(arm: ArmClient):
    log = arm.get_logger()
    arm.go_home("初始归位")
    # arm.gripper(0, label="初始-夹爪闭合")   # 确保初始闭合

    # for idx, name in enumerate(VISIT_ORDER, 1):
    #     dx, dy = HOLE_OFFSETS[name]
    #     hx = B_X + dx
    #     hy = B_Y + dy
    #     hz = B_Z

    #     log.info(
    #         f"\n{'─'*50}\n"
    #         f"[{idx}/{len(VISIT_ORDER)}] 孔位 {name}"
    #         f"  abs=({hx:.4f}, {hy:.4f}, {hz:.4f})\n"
    #         f"{'─'*50}"
    #     )

    #     # 1. 快速到孔正上方（夹爪保持闭合）
    #     ok = arm.move(hx, hy, hz + APPROACH_OFFSET_Z,
    #                   _R, _P, _Y, SPEED_FAST, ACCEL_FAST, f"孔{name}-接近")
    #     if not ok:
    #         log.error(f"孔 {name} 接近失败，跳过")
    #         continue

    #     # 2. 慢速下降到孔面（夹爪保持闭合）
    #     ok = arm.move(hx, hy, hz, _R, _P, _Y, SPEED_SLOW, ACCEL_SLOW, f"孔{name}-到位")
    #     if not ok:
    #         log.error(f"孔 {name} 到位失败，退出")
    #         break

    #     # # 3. 到位后夹爪开 10%
    #     # arm.gripper(100, label=f"孔{name}-开10%")

    #     # # 4. 抬回接近高度，夹爪关闭备用
    #     # arm.gripper(0, label=f"孔{name}-夹爪复位")
    #     arm.move(hx, hy, hz + APPROACH_OFFSET_Z,
    #              _R, _P, _Y, SPEED_FAST, ACCEL_FAST, f"孔{name}-抬起")

    # # arm.go_home("完成归位")
    # log.info("=" * 50)
    # log.info("  ✓ 所有孔位访问完成！")
    # log.info("=" * 50)


def main():
    rclpy.init()
    arm = ArmClient()
    try:
        visit_holes(arm)
    finally:
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
