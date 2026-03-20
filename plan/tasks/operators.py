"""
power_panel 安装任务 - HTN operators（原子动作）

动作与感知的对应关系：
  inspect_bin        -> perception/bin_detector.py       (step1.png)
  pick_power_panel   -> 机器人抓取
  move_to_backplane  -> 机器人移动
  place_power_panel  -> 机器人放置
  verify_placement   -> perception/panel_detector.py     (step2.png)
  detect_screw_holes -> perception/screw_detector.py     (step3.png)
  fetch_screw        -> 机器人取螺丝
  locating_screw     -> 机器人对孔
  insert_screw       -> 机器人插入
  fasten_screw       -> 机器人拧紧
"""

import sys
from pathlib import Path

_HTN_ROOT = Path(__file__).parent.parent / "htn"
if str(_HTN_ROOT) not in sys.path:
    sys.path.insert(0, str(_HTN_ROOT))

from robot_htn import htn


# ---------------------------------------------------------------------------
# Step 1: 料箱检测 & 抓取
# ---------------------------------------------------------------------------

def inspect_bin(state):
    """用 YOLO 检测料箱中是否有 power_panel"""
    if state.robot_at == "bin" and not state.bin_inspected:
        state.bin_has_panel = state._perception_bin_result   # 由调用方注入
        state.bin_inspected = True
        return state
    return False


def pick_power_panel(state):
    """从料箱抓取 power_panel"""
    if (state.bin_inspected
            and state.bin_has_panel
            and not state.holding_panel
            and state.robot_at == "bin"):
        state.holding_panel = True
        state.bin_has_panel = False
        return state
    return False


# ---------------------------------------------------------------------------
# Step 2: 移动 & 放置 & 验证
# ---------------------------------------------------------------------------

def move_to_backplane(state):
    """移动到背板安装位置"""
    if state.holding_panel:
        state.robot_at = "backplane"
        return state
    return False


def move_to_bin(state):
    """移动到料箱位置"""
    if not state.holding_panel:
        state.robot_at = "bin"
        return state
    return False


def place_power_panel(state):
    """将 power_panel 放置到背板"""
    if state.holding_panel and state.robot_at == "backplane":
        state.holding_panel = False
        state.panel_placed = True
        return state
    return False


def verify_placement(state):
    """用 YOLO 验证 power_panel 是否安装到位"""
    if state.panel_placed and not state.placement_verified:
        state.placement_ok = state._perception_panel_result  # 由调用方注入
        state.placement_verified = True
        return state
    return False


def repick_power_panel(state):
    """放置失败时重新抓取"""
    if (state.panel_placed
            and state.placement_verified
            and not state.placement_ok
            and state.robot_at == "backplane"):
        state.holding_panel = True
        state.panel_placed = False
        state.placement_verified = False
        return state
    return False


# ---------------------------------------------------------------------------
# Step 3: 螺丝孔检测 & 安装
# ---------------------------------------------------------------------------

def detect_screw_holes(state):
    """用 YOLO 检测螺丝孔位置"""
    if state.placement_ok and not state.holes_detected:
        state.holes_detected = True
        state.hole_positions = state._perception_screw_result  # dict hole_name -> (cx,cy)
        return state
    return False


def fetch_screw(state):
    """取一颗螺丝"""
    if (state.holes_detected
            and not state.holding_screw
            and state.screws_available > 0):
        state.holding_screw = True
        state.screws_available -= 1
        return state
    return False


def locating_screw(state, hole):
    """将螺丝对准指定螺孔"""
    if (state.holding_screw
            and state.placement_ok
            and hole not in state.screws_fastened):
        state.screw_aligned_to = hole
        return state
    return False


def insert_screw(state, hole):
    """将螺丝插入螺孔"""
    if state.screw_aligned_to == hole:
        state.screw_inserted_at = hole
        state.screw_aligned_to = None
        return state
    return False


def fasten_screw(state, hole):
    """拧紧螺丝"""
    if state.screw_inserted_at == hole:
        state.screws_fastened.append(hole)
        state.holding_screw = False
        state.screw_inserted_at = None
        return state
    return False


# ---------------------------------------------------------------------------
# 注册
# ---------------------------------------------------------------------------

htn.declare_operators(
    inspect_bin,
    pick_power_panel,
    move_to_backplane,
    move_to_bin,
    place_power_panel,
    verify_placement,
    repick_power_panel,
    detect_screw_holes,
    fetch_screw,
    locating_screw,
    insert_screw,
    fasten_screw,
)
