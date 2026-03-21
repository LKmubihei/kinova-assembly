"""
power_panel 安装任务 - HTN methods（任务分解）

任务层次：
  install_power_panel
    ├── check_and_pick          (step1: 检查料箱 -> 抓取)
    ├── place_and_verify        (step2: 放置 -> 验证，失败则重试)
    └── install_all_screws      (step3: 检测孔位 -> 逐个安装螺丝)
         └── tighten_one_screw  (取螺丝 -> 对孔 -> 插入 -> 拧紧)
"""

import sys
from pathlib import Path

_HTN_ROOT = Path(__file__).parent.parent / "htn"
if str(_HTN_ROOT) not in sys.path:
    sys.path.insert(0, str(_HTN_ROOT))

from robot_htn import htn

# 螺丝安装顺序：A->B->C->D 固定，E/F/G 随后
_ORDERED_HOLES = ["hole_A", "hole_B", "hole_C", "hole_D"]
_EXTRA_HOLES   = ["hole_E", "hole_F", "hole_G"]
_ALL_HOLES     = _ORDERED_HOLES + _EXTRA_HOLES


# ===========================================================================
# 顶层任务: install_power_panel
# ===========================================================================

def install_blocked(state):
    if state.manual_review_required:
        return []
    return False

def install_already_done(state):
    if state.placement_ok and set(_ALL_HOLES) == set(state.screws_fastened):
        return []
    return False


def install_full(state):
    if not state.bin_inspected and not state.holding_panel:
        return [
            ("check_and_pick",),
            ("place_and_verify",),
            ("install_all_screws",),
        ]
    return False


def install_resume_holding(state):
    """断点续做：已持有 panel，跳过抓取"""
    if state.holding_panel:
        return [
            ("place_and_verify",),
            ("install_all_screws",),
        ]
    return False


def install_resume_placed(state):
    """断点续做：已放置但未验证"""
    if state.panel_placed and not state.placement_verified:
        return [
            ("place_and_verify",),
            ("install_all_screws",),
        ]
    return False


def install_resume_screws(state):
    """断点续做：已验证放置，直接安装螺丝"""
    if state.placement_ok and not set(_ALL_HOLES) == set(state.screws_fastened):
        return [("install_all_screws",)]
    return False


htn.declare_methods(
    "install_power_panel",
    install_blocked,
    install_already_done,
    install_resume_holding,
    install_resume_placed,
    install_resume_screws,
    install_full,
)


# ===========================================================================
# Step 1: check_and_pick
# ===========================================================================

def check_and_pick_blocked(state):
    if state.manual_review_required or state.holding_panel:
        return []
    return False

def check_and_pick_method(state):
    if state.manual_review_required:
        return False
    subtasks = []
    if state.robot_at != "bin":
        subtasks.append(("move_to_bin",))
    if not state.bin_inspected:
        subtasks.append(("inspect_bin",))
    subtasks.append(("resolve_bin_result",))
    return subtasks


htn.declare_methods("check_and_pick", check_and_pick_blocked, check_and_pick_method)


def resolve_bin_result_blocked(state):
    if state.manual_review_required:
        return []
    return False


def resolve_bin_result_pick(state):
    if state.bin_inspected and state.bin_has_panel and not state.holding_panel:
        return [("pick_power_panel",)]
    return False


def resolve_bin_result_manual(state):
    if state.bin_inspected and not state.bin_has_panel:
        return [("request_manual_review", "料箱未检测到 power_panel，请检查来料或人工确认")]
    return False


htn.declare_methods(
    "resolve_bin_result",
    resolve_bin_result_blocked,
    resolve_bin_result_pick,
    resolve_bin_result_manual,
)


# ===========================================================================
# Step 2: place_and_verify（含重试）
# ===========================================================================

def place_and_verify_blocked(state):
    if state.manual_review_required:
        return []
    return False


def place_and_verify_done(state):
    if state.placement_verified and state.placement_ok:
        return []
    return False

def place_and_verify_fresh(state):
    """持有 panel，移动 -> 放置 -> 验证"""
    if state.holding_panel:
        return [
            ("move_to_backplane",),
            ("place_power_panel",),
            ("verify_placement",),
            ("handle_placement_result",),
        ]
    return False


def place_and_verify_placed(state):
    """已放置但未验证"""
    if state.panel_placed and not state.placement_verified:
        return [
            ("verify_placement",),
            ("handle_placement_result",),
        ]
    return False

htn.declare_methods(
    "place_and_verify",
    place_and_verify_blocked,
    place_and_verify_done,
    place_and_verify_fresh,
    place_and_verify_placed,
)


def handle_placement_ok(state):
    """验证通过，继续"""
    if state.placement_verified and state.placement_ok:
        return []
    return False


def handle_placement_blocked(state):
    if state.manual_review_required:
        return []
    return False


def handle_placement_retry(state):
    """验证失败，在重试上限内重新抓取并重试"""
    if (
        state.placement_verified
        and not state.placement_ok
        and state.placement_attempts < state.max_placement_attempts
    ):
        return [
            ("repick_power_panel",),
            ("place_and_verify",),
        ]
    return False


def handle_placement_manual(state):
    """验证失败且达到上限时，转入人工复核"""
    if (
        state.placement_verified
        and not state.placement_ok
        and state.placement_attempts >= state.max_placement_attempts
    ):
        reason = (
            f"放置验证连续失败 {state.placement_attempts} 次，"
            "请人工确认安装状态后重新规划"
        )
        return [("request_manual_review", reason)]
    return False


htn.declare_methods(
    "handle_placement_result",
    handle_placement_blocked,
    handle_placement_ok,
    handle_placement_retry,
    handle_placement_manual,
)


# ===========================================================================
# Step 3: install_all_screws
# ===========================================================================

def install_all_screws_blocked(state):
    if state.manual_review_required:
        return []
    return False


def install_all_screws_done(state):
    if set(_ALL_HOLES) == set(state.screws_fastened):
        return []
    return False

def install_all_screws_method(state):
    if not state.placement_ok:
        return False
    subtasks = []
    if not state.holes_detected:
        subtasks.append(("detect_screw_holes",))
    subtasks.append(("resolve_screw_holes",))
    return subtasks


htn.declare_methods(
    "install_all_screws",
    install_all_screws_blocked,
    install_all_screws_done,
    install_all_screws_method,
)


def resolve_screw_holes_blocked(state):
    if state.manual_review_required:
        return []
    return False


def resolve_screw_holes_manual(state):
    if state.holes_detected and state.missing_holes:
        reason = (
            "螺丝孔识别不完整："
            + ", ".join(state.missing_holes)
            + "，请人工修正孔位后重新规划"
        )
        return [("request_manual_review", reason)]
    return False


def resolve_screw_holes_install(state):
    if not state.holes_detected or state.missing_holes:
        return False
    remaining = [hole for hole in _ALL_HOLES if hole not in state.screws_fastened]
    return [("tighten_one_screw", hole) for hole in remaining]


htn.declare_methods(
    "resolve_screw_holes",
    resolve_screw_holes_blocked,
    resolve_screw_holes_manual,
    resolve_screw_holes_install,
)


def tighten_already_done(state, hole):
    if state.manual_review_required:
        return []
    if hole in state.screws_fastened:
        return []
    return False


def tighten_one_screw_method(state, hole):
    if (
        state.holes_detected
        and hole in state.hole_positions
        and hole not in state.screws_fastened
        and not state.holding_screw
        and not state.manual_review_required
    ):
        return [
            ("fetch_screw",),
            ("locating_screw", hole),
            ("insert_screw", hole),
            ("fasten_screw", hole),
        ]
    return False


htn.declare_methods("tighten_one_screw", tighten_already_done, tighten_one_screw_method)
