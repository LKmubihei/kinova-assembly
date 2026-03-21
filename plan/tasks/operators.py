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

EXPECTED_HOLES = tuple(f"hole_{c}" for c in "ABCDEFG")


def _get_panel_verification_result(state) -> bool:
    sequence = list(getattr(state, "_perception_panel_result_sequence", []))
    if sequence:
        index = min(state.placement_attempts, len(sequence) - 1)
        return bool(sequence[index])
    return bool(getattr(state, "_perception_panel_result", False))


# ---------------------------------------------------------------------------
# Step 1: 料箱检测 & 抓取
# ---------------------------------------------------------------------------

def inspect_bin(state):
    """用 YOLO 检测料箱中是否有 power_panel"""
    if state.manual_review_required:
        return False
    if state.robot_at == "bin" and not state.bin_inspected:
        state.bin_has_panel = bool(state._perception_bin_result)
        state.bin_inspected = True
        state.workflow_status = "料箱检测完成"
        return state
    return False


def pick_power_panel(state):
    """从料箱抓取 power_panel"""
    if state.manual_review_required:
        return False
    if (
        state.bin_inspected
        and state.bin_has_panel
        and not state.holding_panel
        and state.robot_at == "bin"
    ):
        state.holding_panel = True
        state.bin_has_panel = False
        state.workflow_status = "已抓取 power_panel"
        return state
    return False


# ---------------------------------------------------------------------------
# Step 2: 移动 & 放置 & 验证
# ---------------------------------------------------------------------------

def move_to_backplane(state):
    """移动到背板安装位置"""
    if state.manual_review_required:
        return False
    if state.holding_panel:
        state.robot_at = "backplane"
        state.workflow_status = "已到达背板工位"
        return state
    return False


def move_to_bin(state):
    """移动到料箱位置"""
    if state.manual_review_required:
        return False
    if not state.holding_panel:
        state.robot_at = "bin"
        state.workflow_status = "已到达料箱工位"
        return state
    return False


def place_power_panel(state):
    """将 power_panel 放置到背板"""
    if state.manual_review_required:
        return False
    if state.holding_panel and state.robot_at == "backplane":
        state.holding_panel = False
        state.panel_placed = True
        state.workflow_status = "已执行放置"
        return state
    return False


def verify_placement(state):
    """用 YOLO 验证 power_panel 是否安装到位"""
    if state.manual_review_required:
        return False
    if state.panel_placed and not state.placement_verified:
        state.placement_ok = _get_panel_verification_result(state)
        state.placement_verified = True
        state.placement_attempts += 1
        if state.placement_ok:
            state.workflow_status = f"第 {state.placement_attempts} 次放置验证通过"
        else:
            state.workflow_status = f"第 {state.placement_attempts} 次放置验证失败"
        return state
    return False


def repick_power_panel(state):
    """放置失败时重新抓取"""
    if (
        state.panel_placed
        and state.placement_verified
        and not state.placement_ok
        and state.robot_at == "backplane"
        and not state.manual_review_required
    ):
        state.holding_panel = True
        state.panel_placed = False
        state.placement_verified = False
        state.placement_ok = False
        state.workflow_status = "放置失败，已回收工件准备重试"
        return state
    return False


# ---------------------------------------------------------------------------
# Step 3: 螺丝孔检测 & 安装
# ---------------------------------------------------------------------------

def detect_screw_holes(state):
    """用 YOLO 检测螺丝孔位置"""
    if state.manual_review_required:
        return False
    if state.placement_ok and not state.holes_detected:
        state.holes_detected = True
        state.hole_positions = dict(state._perception_screw_result)
        state.missing_holes = [
            hole for hole in state.expected_holes if hole not in state.hole_positions
        ]
        if state.missing_holes:
            state.workflow_status = "孔位识别不完整"
        else:
            state.workflow_status = "孔位识别完成"
        return state
    return False


def fetch_screw(state):
    """取一颗螺丝"""
    if (
        state.holes_detected
        and state.placement_ok
        and not state.manual_review_required
        and not state.holding_screw
        and state.screws_available > 0
    ):
        state.holding_screw = True
        state.screws_available -= 1
        state.workflow_status = "已取一颗螺丝"
        return state
    return False


def locating_screw(state, hole):
    """将螺丝对准指定螺孔"""
    if (
        state.holding_screw
        and state.placement_ok
        and not state.manual_review_required
        and hole in state.hole_positions
        and hole not in state.screws_fastened
    ):
        state.screw_aligned_to = hole
        state.workflow_status = f"已对准 {hole}"
        return state
    return False


def insert_screw(state, hole):
    """将螺丝插入螺孔"""
    if state.manual_review_required:
        return False
    if state.screw_aligned_to == hole:
        state.screw_inserted_at = hole
        state.screw_aligned_to = None
        state.workflow_status = f"已插入 {hole}"
        return state
    return False


def fasten_screw(state, hole):
    """拧紧螺丝"""
    if state.manual_review_required:
        return False
    if state.screw_inserted_at == hole:
        state.screws_fastened.append(hole)
        state.holding_screw = False
        state.screw_inserted_at = None
        state.workflow_status = f"已拧紧 {hole}"
        return state
    return False


def request_manual_review(state, reason):
    """进入人工复核分支，阻断后续自动装配。"""
    if not state.manual_review_required:
        state.manual_review_required = True
        state.manual_review_reason = str(reason)
        state.workflow_status = "需要人工复核"
        return state
    if state.manual_review_reason != str(reason):
        state.manual_review_reason = str(reason)
    return state


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
    request_manual_review,
)
