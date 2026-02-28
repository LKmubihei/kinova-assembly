#!/usr/bin/env python3
"""
抓取绿色电池脚本
BX / BY 由 Gazebo 实时位置动态计算得到：
  1. 用 `gz model` 读取 green_battery 和 gen3 在 Gazebo world 中的位置
  2. 计算 battery 相对于 robot base 的偏移（world frame）
  3. 绕 robot yaw 的逆旋转，转换到 robot base_link 坐标系
"""

import subprocess
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from hyy_message.action import MoveXYZW
from hyy_message.srv import Setangle
import time
import math


# ── 从 Gazebo 实时读取位置，计算电池在 base_link 中的 (x, y) ──────────
def _gz_pose(model_name: str):
    """返回 (x, y, z, roll, pitch, yaw)，单位 m / rad。"""
    result = subprocess.run(
        ['gz', 'model', '-m', model_name, '-p'],
        capture_output=True, text=True, timeout=5
    )
    if result.returncode != 0:
        raise RuntimeError(f'gz model 获取 {model_name} 失败: {result.stderr.strip()}')
    values = [float(v) for v in result.stdout.split()]
    return values  # [x, y, z, roll, pitch, yaw]


def compute_battery_in_base_link(battery_model='green_battery', robot_model='gen3',
                                  collision_z_offset=0.01):
    """
    返回电池碰撞中心在机器人 base_link 坐标系中的 (x, y, z)。

    参数:
        battery_model      : Gazebo 中电池模型名称
        robot_model        : Gazebo 中机器人模型名称
        collision_z_offset : 电池 SDF 中碰撞中心相对 model origin 的 z 偏移（见 model.sdf）
    """
    bx, by, bz, *_ = _gz_pose(battery_model)
    rx, ry, rz, _, _, ryaw = _gz_pose(robot_model)

    # 电池碰撞中心在 Gazebo world 中
    bz_center = bz + collision_z_offset

    # 相对于机器人底座的偏移（world frame）
    dx = bx - rx
    dy = by - ry
    dz = bz_center - rz

    # 逆旋转（将 world 偏移转换到 robot base_link frame）
    cos_a = math.cos(-ryaw)
    sin_a = math.sin(-ryaw)
    x_local = cos_a * dx - sin_a * dy
    y_local = sin_a * dx + cos_a * dy
    z_local = dz

    print(f'[位置计算] Gazebo world — battery:({bx:.3f},{by:.3f},{bz:.3f})  '
          f'robot:({rx:.3f},{ry:.3f},{rz:.3f}) yaw={math.degrees(ryaw):.1f}°')
    print(f'[位置计算] base_link frame — x={x_local:.4f}, y={y_local:.4f}, z={z_local:.4f}')
    return x_local, y_local, z_local


# 动态计算电池位置
BX, BY, _bz = compute_battery_in_base_link(collision_z_offset=0.02)
BZ_CENTER = max(_bz + 0.011, 0.010)  # 碰撞中心加安全余量，不低于 0.01m

# 姿态：夹爪朝下（与当前机械臂状态一致）
ROLL  = -math.pi   # -3.14159
PITCH = 0.0
YAW   = 0.0

SPEED = 0.2
ACCEL = 0.10

class Move_cmd:
    def __init__(self, x, y, z, roll, pitch, yaw, speed, accel):
        self.positionx = x
        self.positiony = y
        self.positionz = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.speed = speed
        self.accel = accel

class Gripper_cmd:
    def __init__(self, position, force=50):
        self.position = position  # 0=closed, 1000=open
        self.force = force

class MoveXYZWClient(Node):
    def __init__(self):
        super().__init__('battery_grasp_client')
        self._ac = ActionClient(self, MoveXYZW, 'MoveXYZW')
        self.get_logger().info('等待 MoveXYZW action server...')
        self._ac.wait_for_server()
        self.get_logger().info('已连接 MoveXYZW action server')

    def send_goal(self, cmd, label=''):
        if not (0 < cmd.speed <= 0.2):
            cmd.speed = 0.05
        if not (0 < cmd.accel <= 0.2):
            cmd.accel = 0.03

        goal = MoveXYZW.Goal()
        goal.positionx = cmd.positionx
        goal.positiony = cmd.positiony
        goal.positionz = cmd.positionz
        goal.yaw   = cmd.yaw
        goal.pitch = cmd.pitch
        goal.roll  = cmd.roll
        goal.speed = cmd.speed
        goal.accel = cmd.accel

        self.get_logger().info(
            f'[{label}] 目标: x={goal.positionx:.3f}, y={goal.positiony:.3f}, '
            f'z={goal.positionz:.3f} | RPY=({goal.roll:.2f},{goal.pitch:.2f},{goal.yaw:.2f})'
        )

        future = self._ac.send_goal_async(goal, feedback_callback=self._fb)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(f'[{label}] Goal 被拒绝！')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        ok = result.result == 'MoveXYZW:SUCCESS'
        if ok:
            self.get_logger().info(f'[{label}] ✅ 成功')
        else:
            self.get_logger().error(f'[{label}] ❌ 失败: {result.result}')
        return ok

    def _fb(self, fb):
        pass  # 静默反馈


class GripperClient(Node):
    def __init__(self):
        super().__init__('battery_gripper_client')
        self.cli = self.create_client(Setangle, '/gripper_cmd_Node/ur_gripper_cmd_server')
        self.get_logger().info('等待夹爪服务...')
        self.cli.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('已连接夹爪服务')

    def send(self, cmd, label=''):
        req = Setangle.Request()
        req.status = 'gripper_cmd'
        req.hand_id = 1
        req.angle = [int(cmd.force), int(cmd.position)]
        future = self.cli.call_async(req)
        self.get_logger().info(
            f'[{label}] 夹爪: position={cmd.position}, force={cmd.force}'
        )
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        resp = future.result()
        if resp and resp.angle_accepted:
            self.get_logger().info(f'[{label}] ✅ 夹爪指令接受')
        else:
            self.get_logger().error(f'[{label}] ❌ 夹爪指令拒绝')


def main():
    rclpy.init()
    arm = MoveXYZWClient()
    gripper = GripperClient()

    # ── 目标姿态定义 ──────────────────────────────────────────
    # 0) 安全初始位姿（在电池正上方较高处）
    pose_above_far = Move_cmd(BX, BY, 0.30,  ROLL, PITCH, YAW, SPEED, ACCEL)

    # 1) 预接近（电池正上方 15 cm）
    pose_pre  = Move_cmd(BX, BY, 0.15,     ROLL, PITCH, YAW, SPEED, ACCEL)

    # 2) 抓取位（grasp_point 降至电池中心高度）
    pose_grasp = Move_cmd(BX, BY, BZ_CENTER, ROLL, PITCH, YAW, SPEED*0.7, ACCEL*0.7)

    # 3) 抬起（夹住后提升）
    pose_lift  = Move_cmd(BX, BY, 0.35,    ROLL, PITCH, YAW, SPEED, ACCEL)

    gripper_open  = Gripper_cmd(position=1000, force=50)
    gripper_close = Gripper_cmd(position=0,    force=80)   # 0 = 全关，遇到物体时自动停止

    # ── 执行序列 ──────────────────────────────────────────────
    print('\n' + '='*60)
    print('  开始抓取绿色电池')
    print(f'  目标位置 (base_link): x={BX:.3f}, y={BY:.3f}')
    print('='*60 + '\n')

    # Step 0: 移动到电池上方安全高度
    arm.send_goal(pose_above_far, '0-安全高度')
    time.sleep(0.5)

    # Step 1: 打开夹爪
    gripper.send(gripper_open, '1-张开夹爪')
    time.sleep(0.5)

    # Step 2: 预接近（靠近电池上方）
    if not arm.send_goal(pose_pre, '2-预接近'):
        print('❌ 预接近失败，停止')
        arm.destroy_node()
        gripper.destroy_node()
        rclpy.shutdown()
        return

    time.sleep(0.3)

    # Step 3: 缓慢下降到抓取高度
    if not arm.send_goal(pose_grasp, '3-下降抓取'):
        print('❌ 下降到抓取位置失败，尝试退回')
        arm.send_goal(pose_above_far, '退回-安全高度')
        arm.destroy_node()
        gripper.destroy_node()
        rclpy.shutdown()
        return

    time.sleep(0.5)

    # Step 4: 单步关闭夹爪
    # PID(p=3.0) 限制闭合速度；allow_stalling 在接触电池时自动停止；
    # grasp_fix 检测到双侧接触后附着电池（disable_collisions_on_attach）。
    # 附着后不再发任何夹爪指令，避免穿透导致脱落。
    gripper.send(gripper_close, '4-关闭夹爪')
    time.sleep(1.0)  # 等待 grasp_fix 完成附着

    # Step 5: 抬起
    print('\n>>> 抬起电池 <<<')
    arm.send_goal(pose_lift, '5-抬起')
    time.sleep(0.5)

    print('\n' + '='*60)
    print('  ✅ 抓取序列完成！')
    print('='*60 + '\n')

    arm.destroy_node()
    gripper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
