#!/usr/bin/env python3
"""
TV装配执行器 (Assembly Executor)
================================
流程：
  1. unified_planning + fast-downward 求解 PDDL 计划
  2. 逐步执行计划中每个符号动作
  3. 每个动作：
       ① YOLO 检测接口（桩函数，保留接口，后续替换真实推理）
       ② 机械臂移动  (MoveXYZW action)
       ③ 夹爪控制    (Setangle service)

设计说明：
  - YOLOInterface   : 所有检测桩，标注 TODO 供后续接入真实模型
  - AssemblyRobot   : ROS2 节点，封装机械臂移动 + 夹爪控制
  - AssemblyExecutor: 将 PDDL 动作名映射到具体执行逻辑
  - solve_plan()    : 调用 PDDL 求解器，返回动作序列

位置配置：
  修改 POSES 字典以适配实际标定结果（单位：m / rad）
"""

import time
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from hyy_message.action import MoveXYZW
from hyy_message.srv import Setangle

import unified_planning as up
from unified_planning.shortcuts import OneshotPlanner
from unified_planning.io import PDDLReader

# 关闭规划引擎 credits 打印
up.shortcuts.get_environment().credits_stream = None

# PDDL 文件路径
DOMAIN_FILE  = "/home/lk/workspace/src/plan/domain.pddl"
PROBLEM_FILE = "/home/lk/workspace/src/plan/p_real.pddl"

# =============================================================================
# 位姿配置表
# key -> (x, y, z, roll, pitch, yaw, speed, accel)  单位: m / rad
# 实际部署时根据手眼标定结果修改此处的数值
# =============================================================================
_RD = -math.pi   # roll_down: 夹爪朝下
_P0 = 0.0
_Y0 = 0.0
_SF, _AF = 0.20, 0.10   # 快速移动
_SS, _AS = 0.10, 0.05   # 慢速移动（精定位 / 接近）

POSES: dict[str, tuple] = {
    # ── 全局安全 / 过渡位置 ────────────────────────────────────────
    "home":              (0.30,  0.00,  0.40, _RD, _P0, _Y0, _SF, _AF),

    # ── 料盒 (material_box) ────────────────────────────────────────
    "box_above":         (0.45,  0.20,  0.35, _RD, _P0, _Y0, _SF, _AF),
    "box_inspect":       (0.45,  0.20,  0.22, _RD, _P0, _Y0, _SS, _AS),
    "box_grasp":         (0.45,  0.20,  0.11, _RD, _P0, _Y0, _SS, _AS),

    # ── 面板区域 (TV_panel) ────────────────────────────────────────
    "panel_coarse":      (0.55, -0.10,  0.30, _RD, _P0, _Y0, _SF, _AF),
    "panel_locate":      (0.55, -0.10,  0.18, _RD, _P0, _Y0, _SS, _AS),
    "panel_place":       (0.55, -0.10,  0.07, _RD, _P0, _Y0, _SS, _AS),

    # ── 螺丝供料器 (screw feeder) ──────────────────────────────────
    "feeder_above":      (0.28,  0.38,  0.28, _RD, _P0, _Y0, _SF, _AF),
    "feeder_take":       (0.28,  0.38,  0.12, _RD, _P0, _Y0, _SS, _AS),

    # ── 孔位 A~G  (above: 上方接近；insert: 插入点) ────────────────
    # 间距 25 mm，沿 x 轴排列；可根据实际面板坐标调整
    **{f"hole_{h}_above":  (0.48 + i * 0.025, -0.06,  0.22, _RD, _P0, _Y0, _SF, _AF)
       for i, h in enumerate("ABCDEFG")},
    **{f"hole_{h}_insert": (0.48 + i * 0.025, -0.06,  0.06, _RD, _P0, _Y0, _SS, _AS)
       for i, h in enumerate("ABCDEFG")},
}


# =============================================================================
# YOLO 检测接口（桩函数）
# 每个方法对应 domain.pddl 里一个动作的前 / 后条件判定。
# 实际使用时将 "# TODO" 行替换为真实推理调用。
# =============================================================================
class YOLOInterface:
    """所有 YOLO 检测桩。返回值格式保持不变，内部实现可替换。"""

    # ── 组件检测 ──────────────────────────────────────────────────────────────

    def inspect_component_in_box(self, box_name: str) -> dict:
        """
        [inspect_power_com] 前置判定
        确认料盒中有目标组件，类别正确，姿态可抓。
        返回: {'found': bool, 'class': str, 'bbox': [x,y,w,h], 'pose_ok': bool}
        """
        # TODO: 订阅相机话题，调用 YOLO 推理检测料盒中组件
        return {"found": True, "class": "power_com", "bbox": [100, 120, 80, 60], "pose_ok": True}

    def locate_component_in_box(self, box_name: str) -> dict:
        """
        [pick_power_com] 参数绑定
        在料盒中定位组件精确位置/姿态，输出相对于预设抓取点的偏移。
        返回: {'found': bool, 'grasp_offset': (dx, dy, dz)}
        """
        # TODO: 调用 YOLO 精定位，返回相对于 POSES["box_grasp"] 的偏移
        return {"found": True, "grasp_offset": (0.0, 0.0, 0.0)}

    def align_component_convex_hull(self, comp_name: str, panel_name: str) -> dict:
        """
        [locating_power_com] 主要用途——凸包检测
        识别凸包/定位柱，估算组件与面板的相对位姿误差，用于视觉伺服对齐。
        返回: {'aligned': bool, 'offset': (dx, dy, dz, dyaw)}
        """
        # TODO: 调用 YOLO 凸包/关键点检测，计算对齐偏差，迭代直到 aligned=True
        return {"aligned": True, "offset": (0.0, 0.0, 0.0, 0.0)}

    def verify_component_on_panel(self, comp_name: str, panel_name: str) -> bool:
        """
        [place_power_com] 后置验收
        检查组件是否在位、方向正确、贴合面板。
        """
        # TODO: 检测组件是否正确放在面板上（凸包 / 边界 / 特征对齐）
        return True

    # ── 螺丝检测 ──────────────────────────────────────────────────────────────

    def verify_screw_fetched(self) -> bool:
        """
        [fetch_screw] 后置验收
        确认批头/夹爪上确实有一颗螺丝（防止空抓）。
        """
        # TODO: 检测批头尖端是否有螺丝（亮点 / 轮廓检测）
        return True

    def locate_hole_and_screw(self, screw_name: str, hole_name: str) -> dict:
        """
        [locating_screw] 主要用途——螺孔 + 螺丝检测
        - 螺孔检测：找目标孔的位置/姿态（建议 seg / 关键点）
        - 螺丝检测：找批头螺丝的尖端/轴线
        - 计算两者相对位姿偏差，用于对孔定位
        返回: {'hole_found': bool, 'screw_found': bool, 'offset': (dx, dy, dz)}
        """
        # TODO: 调用 YOLO 螺孔/螺丝检测，计算对孔偏差
        return {"hole_found": True, "screw_found": True, "offset": (0.0, 0.0, 0.0)}

    def verify_screw_fastened(self, screw_name: str, hole_name: str) -> bool:
        """
        [fasten_screw] 后置验收（辅助）
        外观验收：螺丝头是否在位、是否歪斜、是否漏打。
        主判断依靠电批扭矩曲线，以及YOLO 做二次确认。
        """
        # TODO: 检测螺丝头外观（形态 / 角度偏差）
        return True


# =============================================================================
# ROS2 机器人节点：机械臂 + 夹爪
# =============================================================================
class AssemblyRobot(Node):
    """封装 MoveXYZW action 和 Setangle service，提供 move / gripper 接口。"""

    GRIPPER_OPEN  = 1000   # 全张开
    GRIPPER_CLOSE = 0      # 全闭合（遇物自停）

    def __init__(self):
        super().__init__("assembly_executor")
        # 机械臂 action client
        self._arm_ac = ActionClient(self, MoveXYZW, "MoveXYZW")
        # 夹爪 service client
        self._grip_cli = self.create_client(
            Setangle, "/gripper_cmd_Node/ur_gripper_cmd_server"
        )

    def wait_for_services(self, timeout: float = 30.0):
        self.get_logger().info("等待 MoveXYZW action server …")
        self._arm_ac.wait_for_server()
        self.get_logger().info("等待夹爪 service …")
        self._grip_cli.wait_for_service(timeout_sec=timeout)
        self.get_logger().info("机械臂 + 夹爪服务已就绪")

    # ── 机械臂移动 ────────────────────────────────────────────────────────────
    def move(self, pose_key: str, label: str = "") -> bool:
        """按位姿键名移动机械臂。返回 True 表示成功。"""
        if pose_key not in POSES:
            self.get_logger().error(f"[{label}] 未知位姿键: {pose_key}")
            return False

        x, y, z, roll, pitch, yaw, speed, accel = POSES[pose_key]

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
            f"[{label}] → {pose_key}  "
            f"xyz=({x:.3f},{y:.3f},{z:.3f})  "
            f"rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})"
        )

        future = self._arm_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(f"[{label}] Goal 被拒绝")
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        ok = result_future.result().result.result == "MoveXYZW:SUCCESS"
        if ok:
            self.get_logger().info(f"[{label}] ✓ 到位")
        else:
            self.get_logger().error(f"[{label}] ✗ 移动失败")
        return ok

    # ── 夹爪控制 ──────────────────────────────────────────────────────────────
    def _send_gripper(self, position: int, force: int, label: str) -> bool:
        req = Setangle.Request()
        req.status   = "gripper_cmd"
        req.hand_id  = 1
        req.angle    = [int(force), int(position)]

        self.get_logger().info(
            f"[{label}] 夹爪 position={position}, force={force}"
        )
        future = self._grip_cli.call_async(req)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        resp = future.result()
        ok = resp is not None and resp.angle_accepted
        if ok:
            self.get_logger().info(f"[{label}] ✓ 夹爪指令接受")
        else:
            self.get_logger().error(f"[{label}] ✗ 夹爪指令拒绝")
        return ok

    def open_gripper(self, label: str = "open") -> bool:
        return self._send_gripper(self.GRIPPER_OPEN, 50, label)

    def close_gripper(self, force: int = 80, label: str = "close") -> bool:
        return self._send_gripper(self.GRIPPER_CLOSE, force, label)


# =============================================================================
# 装配执行器：将 PDDL 动作名映射到机器人动作序列
# =============================================================================
class AssemblyExecutor:
    """
    执行 PDDL 求解器输出的动作序列。

    每个 execute_* 方法对应 domain.pddl 中的一个动作：
      1. [可选] YOLO 前置检测
      2. 机械臂 + 夹爪动作序列
      3. [可选] YOLO 后置验收
    """

    def __init__(self, robot: AssemblyRobot, yolo: YOLOInterface):
        self.robot = robot
        self.yolo  = yolo
        self.log   = robot.get_logger()

    def _pause(self, sec: float = 0.5):
        time.sleep(sec)

    # ── 动作分发 ──────────────────────────────────────────────────────────────
    def execute(self, action_name: str, params: list[str]):
        """根据 action_name 分发到对应的执行方法。"""
        self.log.info(
            f"\n{'─'*60}\n▶ 执行动作: {action_name}({', '.join(params)})\n{'─'*60}"
        )
        dispatch = {
            "inspect_power_com":   self._exec_inspect_power_com,
            "pick_power_com":      self._exec_pick_power_com,
            "move_power_com":      self._exec_move_power_com,
            "locating_power_com":  self._exec_locating_power_com,
            "place_power_com":     self._exec_place_power_com,
            "repick_power_com":    self._exec_repick_power_com,
            "fetch_screw":         self._exec_fetch_screw,
            "locating_screw":      self._exec_locating_screw,
            "insert_screw":        self._exec_insert_screw,
            "fasten_screw":        self._exec_fasten_screw,
        }
        fn = dispatch.get(action_name)
        if fn is None:
            self.log.error(f"未知动作: {action_name}")
            return
        fn(*params)

    # =========================================================================
    # 组件放置阶段
    # =========================================================================

    def _exec_inspect_power_com(self):
        """
        [inspect_power_com]
        YOLO：确认料盒中有目标组件，类别正确，可抓取姿态。
        机械臂：移至料盒上方检视位，无需夹爪操作。
        """
        # 1. 移至检视位
        self.robot.move("box_inspect", "检视料盒")
        self._pause(0.5)

        # 2. YOLO 检测
        result = self.yolo.inspect_component_in_box("material_box")
        if result["found"] and result["pose_ok"]:
            self.log.info(f"  ✓ YOLO 检测到组件 class={result['class']}  bbox={result['bbox']}")
        else:
            self.log.warn("  ⚠ YOLO 未检测到有效组件，继续执行（仿真模式）")

        # 3. 抬回上方
        self.robot.move("box_above", "检视后抬起")

    def _exec_pick_power_com(self, comp: str, box: str):
        """
        [pick_power_com]
        YOLO：定位料盒中组件精确位置（grasp_offset）。
        机械臂：移至抓取点，张爪 → 下降 → 闭爪 → 抬起。
        YOLO后置：验证组件已在夹爪中。
        """
        # 1. YOLO 定位
        loc = self.yolo.locate_component_in_box(box)
        dx, dy, dz = loc["grasp_offset"]
        self.log.info(f"  YOLO grasp_offset: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
        # TODO: 将 offset 叠加到 POSES["box_grasp"] 的 xyz 后再发送（此处直接用预设点）

        # 2. 移到料盒上方
        self.robot.move("box_above", f"移至料盒上方[{comp}]")

        # 3. 张开夹爪
        self.robot.open_gripper(label="pick-张爪")
        self._pause(0.3)

        # 4. 下降到抓取高度
        self.robot.move("box_grasp", f"下降抓取[{comp}]")
        self._pause(0.5)

        # 5. 闭合夹爪
        self.robot.close_gripper(force=80, label="pick-夹取")
        self._pause(0.8)

        # 6. 抬起
        self.robot.move("box_above", f"抬起[{comp}]")
        self._pause(0.3)

    def _exec_move_power_com(self, comp: str, panel: str):
        """
        [move_power_com]
        机械臂：从料盒区域粗移动到面板区域上方（不需要精确对齐）。
        YOLO（可选）：若面板位置有漂移，可做粗校正。
        """
        # 移至面板粗定位点
        self.robot.move("home", f"过渡-home[{comp}]")
        self.robot.move("panel_coarse", f"移至面板区域[{comp}→{panel}]")

    def _exec_locating_power_com(self, comp: str, panel: str):
        """
        [locating_power_com]
        YOLO（凸包检测主战场）：识别凸包/定位柱，估计位姿误差，视觉伺服对齐。
        机械臂：下降到精定位高度，根据 YOLO offset 微调位置。
        """
        # 1. 下降到精定位高度
        self.robot.move("panel_locate", f"精定位接近[{comp}]")
        self._pause(0.5)

        # 2. YOLO 凸包检测（迭代视觉伺服）
        for iteration in range(3):
            result = self.yolo.align_component_convex_hull(comp, panel)
            dx, dy, dz, dyaw = result["offset"]
            self.log.info(
                f"  YOLO 对齐 iter={iteration+1}: "
                f"offset=({dx:.3f},{dy:.3f},{dz:.3f},{dyaw:.3f})  aligned={result['aligned']}"
            )
            # TODO: 将 offset 叠加到当前末端位置，发送微调运动指令
            if result["aligned"]:
                break
            self._pause(0.3)

        if result["aligned"]:
            self.log.info(f"  ✓ 组件 {comp} 精定位完成")
        else:
            self.log.warn(f"  ⚠ 精定位未完全收敛，继续（仿真模式）")

    def _exec_place_power_com(self, comp: str, panel: str):
        """
        [place_power_com]
        机械臂：缓慢下降到放置位，张爪放下组件，抬起离开。
        YOLO后置：验证组件在位、方向正确、贴合。
        """
        # 1. 下降到放置位
        self.robot.move("panel_place", f"下降放置[{comp}]")
        self._pause(0.5)

        # 2. 张开夹爪放下
        self.robot.open_gripper(label="place-松爪")
        self._pause(0.5)

        # 3. 抬起离开
        self.robot.move("panel_coarse", f"放置后抬起[{comp}]")
        self._pause(0.3)

        # 4. YOLO 后置验收
        if self.yolo.verify_component_on_panel(comp, panel):
            self.log.info(f"  ✓ YOLO 确认组件 {comp} 已在面板 {panel} 上")
        else:
            self.log.warn(f"  ⚠ YOLO 未确认放置，继续（仿真模式）")

        # 5. 移至安全位
        self.robot.move("home", "放置后归位")

    def _exec_repick_power_com(self, comp: str, panel: str):
        """
        [repick_power_com]
        触发条件：YOLO 判断组件放置后未对齐 / 翘起。
        机械臂：重新抓取面板上的组件，回到 comp-in-hand 状态。
        """
        # 1. 移至面板上方
        self.robot.move("panel_coarse", f"返工接近[{comp}]")

        # 3. 张爪
        self.robot.open_gripper(label="repick-张爪")
        self._pause(0.3)

        # 4. 下降抓取
        self.robot.move("panel_locate", f"返工下降[{comp}]")
        self._pause(0.3)
        self.robot.move("panel_place",  f"返工抓取位[{comp}]")
        self.robot.close_gripper(force=80, label="repick-夹取")
        self._pause(0.8)

        # 5. 抬起
        self.robot.move("panel_coarse", f"返工抬起[{comp}]")
        self.log.info(f"  ✓ 返工抓取完成，将重新执行定位放置")

    # =========================================================================
    # 螺钉紧固阶段
    # =========================================================================

    def _exec_fetch_screw(self):
        """
        [fetch_screw]
        机械臂：移至供料器，取一颗螺丝（闭合夹爪 / 批头就位）。
        YOLO后置：确认批头上有螺丝，防止空抓。
        """
        # 1. 移至供料器上方
        self.robot.move("home", "归位-取螺丝前")
        self.robot.move("feeder_above", "移至供料器上方")

        # 2. 张爪 / 就位
        self.robot.open_gripper(label="feeder-就位张爪")
        self._pause(0.3)

        # 3. 下降取螺丝
        self.robot.move("feeder_take", "下降取螺丝")
        self._pause(0.3)

        # 4. 闭合（批头夹持 / 自动吸附）
        self.robot.close_gripper(force=60, label="feeder-取螺丝")
        self._pause(0.5)

        # 5. 抬起
        self.robot.move("feeder_above", "取螺丝后抬起")

        # 6. YOLO 后置验收
        if self.yolo.verify_screw_fetched():
            self.log.info("  ✓ YOLO 确认已取到螺丝")
        else:
            self.log.warn("  ⚠ YOLO 未检测到螺丝，继续（仿真模式）")

    def _exec_locating_screw(self, screw: str, hole: str):
        """
        [locating_screw]
        YOLO（螺孔 + 螺丝检测主战场）：
          - 螺孔检测：找目标孔位置/姿态
          - 螺丝检测：找批头尖端/轴线
          - 计算相对位姿偏差，对孔定位
        机械臂：移至孔位上方，根据 YOLO offset 精对孔。
        """
        # 提取孔位字母 (e.g. "hole_A" -> "A")
        hole_id = hole.split("_")[-1].upper()

        # 1. 移至孔位上方
        self.robot.move("home", f"归位[{screw}→{hole}]")
        self.robot.move(f"hole_{hole_id}_above", f"移至孔上方[{hole}]")
        self._pause(0.5)

        # 2. YOLO 螺孔 + 螺丝对孔检测
        for iteration in range(3):
            result = self.yolo.locate_hole_and_screw(screw, hole)
            dx, dy, dz = result["offset"]
            self.log.info(
                f"  YOLO 对孔 iter={iteration+1}: "
                f"hole_found={result['hole_found']}  screw_found={result['screw_found']}  "
                f"offset=({dx:.3f},{dy:.3f},{dz:.3f})"
            )
            # TODO: 将 offset 叠加到当前位置，发送微调运动指令
            aligned = abs(dx) < 0.002 and abs(dy) < 0.002
            if aligned:
                break
            self._pause(0.3)

        self.log.info(f"  ✓ 螺丝 {screw} 对孔 {hole} 完成")

    def _exec_insert_screw(self, screw: str, hole: str):
        """
        [insert_screw]
        机械臂：缓慢下降将螺丝送入孔内，完成后松开 / 抬起。
        YOLO后置：检测螺丝是否进入孔。
        """
        hole_id = hole.split("_")[-1].upper()

        # 1. 缓慢下降插入
        self.robot.move(f"hole_{hole_id}_insert", f"插入螺丝[{screw}→{hole}]")
        self._pause(0.5)

        # 2. 松开（批头离开螺丝）
        self.robot.open_gripper(label=f"insert-松开[{screw}]")
        self._pause(0.3)

        # 4. 抬回孔位上方
        self.robot.move(f"hole_{hole_id}_above", f"插入后抬起[{hole}]")

    def _exec_fasten_screw(self, screw: str, hole: str):
        """
        [fasten_screw]
        机械臂：模拟锁付动作（下降→锁付→抬起）。
        YOLO后置（辅助）：外观验收螺丝头是否在位。
        主判断依靠电批扭矩曲线（此处仿真省略）。
        """
        hole_id = hole.split("_")[-1].upper()

        # 1. 下降到孔位（电批就位）
        self.robot.move(f"hole_{hole_id}_insert", f"电批就位[{hole}]")
        self._pause(0.3)

        # 2. 模拟锁付（实际应触发电批旋转，监测扭矩/角度曲线）
        self.log.info(f"  [fasten] 触发电批锁付 {screw} → {hole}  (仿真: 等待 1.0s)")
        self._pause(1.0)

        # 3. 抬起
        self.robot.move(f"hole_{hole_id}_above", f"锁付后抬起[{hole}]")
        self._pause(0.3)

        # 4. YOLO 外观验收
        if self.yolo.verify_screw_fastened(screw, hole):
            self.log.info(f"  ✓ YOLO 确认螺丝 {screw} 已紧固于 {hole}")
        else:
            self.log.warn(f"  ⚠ YOLO 外观验收未通过，继续（仿真模式）")


# =============================================================================
# PDDL 求解
# =============================================================================
def solve_plan() -> list[tuple[str, list[str]]]:
    """
    调用 fast-downward 求解 PDDL 计划。
    返回: [(action_name, [param, ...]), ...]
    """
    reader  = PDDLReader()
    problem = reader.parse_problem(DOMAIN_FILE, PROBLEM_FILE)

    with OneshotPlanner(name="fast-downward", problem_kind=problem.kind) as planner:
        result = planner.solve(problem)

    if result.status not in up.engines.results.POSITIVE_OUTCOMES:
        raise RuntimeError(f"PDDL 求解失败: {result.status}")

    plan_steps = []
    for ai in result.plan.actions:
        name   = ai.action.name
        params = [str(p) for p in ai.actual_parameters]
        plan_steps.append((name, params))

    print("\n" + "=" * 60)
    print("  PDDL 计划（共 {} 步）".format(len(plan_steps)))
    print("=" * 60)
    for i, (name, params) in enumerate(plan_steps, 1):
        print(f"  {i:>2}. {name}({', '.join(params)})")
    print("=" * 60 + "\n")

    return plan_steps


# =============================================================================
# 主函数
# =============================================================================
def main():
    # ── 1. 求解计划（不依赖 ROS2）────────────────────────────────────────────
    plan = solve_plan()

    # ── 2. 初始化 ROS2 ────────────────────────────────────────────────────────
    rclpy.init()
    robot = AssemblyRobot()
    robot.wait_for_services()

    yolo     = YOLOInterface()
    executor = AssemblyExecutor(robot, yolo)

    # ── 3. 归位 ───────────────────────────────────────────────────────────────
    robot.move("home", "初始归位")

    # ── 4. 逐步执行计划 ───────────────────────────────────────────────────────
    for i, (action_name, params) in enumerate(plan, 1):
        print(f"\n[{i}/{len(plan)}] {action_name}({', '.join(params)})")
        executor.execute(action_name, params)

    # ── 5. 归位并退出 ─────────────────────────────────────────────────────────
    robot.move("home", "完成-归位")
    print("\n" + "=" * 60)
    print("  ✓ 装配任务全部完成！")
    print("=" * 60 + "\n")

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
