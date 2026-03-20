"""
电视机电源模块安装 - 任务分解方法定义
TV Power Module Installation - Task Decomposition Methods

Author: HTN Planner
Description: 定义电源模块安装的分层任务网络方法

支持断点续做：机器人可从任意中间状态恢复规划，包括：
- 已持有电源模块（跳过检查和拿取）
- 放置不正确（重试放置）
- 螺丝操作中途（手持螺丝、已对准、已插入）
"""

from robot_htn import htn
import random


# ============================================================================
# 顶层任务 - Top Level Task（多方法支持断点续做）
# ============================================================================

def install_already_done(state, component, target):
    """安装已完成，无需操作"""
    if (state.obj_pos.get(component) == target
            and state.all_screws_done):
        return []
    return False


def install_mid_screw(state, component, target):
    """
    断点续做：螺丝操作中途（手持螺丝）
    先完成当前螺丝操作，再紧固剩余螺丝
    """
    if (state.obj_pos.get(component) == target
            and state.placement_ok == True
            and state.holding == 'screw'):
        return [
            ('continue_current_screw',),
            ('fasten_all_screws',)
        ]
    return False


def install_screws_only(state, component, target):
    """
    断点续做：电源模块已正确放置，仅需紧固螺丝
    """
    if (state.obj_pos.get(component) == target
            and state.placement_ok == True
            and not state.all_screws_done
            and state.holding != 'screw'):
        return [('fasten_all_screws',)]
    return False


def install_retry_placement(state, component, target):
    """
    断点续做：放置不正确，需重试放置后再紧固螺丝
    """
    if (state.obj_pos.get(component) == target
            and state.placement_done == True
            and state.placement_ok == False
            and state.holding == False):
        return [
            ('retry_placement', component, target),
            ('fasten_all_screws',)
        ]
    return False


def install_holding_component(state, component, target):
    """
    断点续做：已持有电源模块，跳过检查和拿取，直接定位放置
    """
    if state.holding == component:
        return [
            ('position_and_place', component, target),
            ('fasten_all_screws',)
        ]
    return False


def install_full_process(state, component, target):
    """
    完整安装流程：从检查拿取开始
    """
    if (state.obj_pos.get(component) != target
            and state.holding == False):
        return [
            ('check_and_pickup', component, 'material_box'),
            ('position_and_place', component, target),
            ('fasten_all_screws',)
        ]
    return False


htn.declare_methods('install_power_module',
                    install_already_done,
                    install_mid_screw,
                    install_screws_only,
                    install_retry_placement,
                    install_holding_component,
                    install_full_process)


# ============================================================================
# 检查和拿取任务 - Check and Pickup Tasks
# ============================================================================

def check_and_pickup_method(state, component, location):
    """
    检查并从料箱拿取电源模块

    根据当前状态动态跳过已完成的步骤（moveto、inspect）
    """
    if state.holding == False and state.obj_pos.get(component) == location:
        subtasks = []

        if state.robot_pos != location:
            subtasks.append(('moveto', location))

        if not state.inspected:
            subtasks.append(('inspect_power_com',))

        subtasks.append(('pick', component, location))

        return subtasks
    return False


htn.declare_methods('check_and_pickup', check_and_pickup_method)


# ============================================================================
# 定位和放置任务 - Position and Place Tasks
# ============================================================================

def position_and_place_success(state, component, target):
    """
    定位并放置电源模块（成功路径）

    根据当前状态跳过已完成的步骤：
    - 已在目标位置 → 跳过 move
    - 已完成定位 → 跳过 locating
    """
    if state.holding == component:
        subtasks = []

        if state.robot_pos != target:
            subtasks.append(('move', component, target))

        if not state.locating_done:
            subtasks.append(('locating', component, target))

        subtasks.append(('place', component, target))

        return subtasks
    return False


def position_and_place_with_retry(state, component, target):
    """
    定位并放置电源模块（带重试路径）
    当放置不正确时，需要重新抓取并二次定位
    """
    if (state.placement_done == True
            and state.placement_ok == False
            and state.obj_pos.get(component) == target):
        return [
            ('retry_placement', component, target)
        ]
    return False


htn.declare_methods('position_and_place',
                    position_and_place_success,
                    position_and_place_with_retry)


def retry_placement_method(state, component, target):
    """
    重试放置任务：重新抓取 → 重新定位 → 重新放置
    """
    if (state.placement_ok == False
            and state.obj_pos.get(component) == target
            and state.holding == False):
        return [
            ('repick', component, target),
            ('locating', component, target),
            ('place', component, target)
        ]
    return False


htn.declare_methods('retry_placement', retry_placement_method)


# ============================================================================
# 继续当前螺丝操作 - Continue Current Screw (断点续做)
# ============================================================================

def continue_screw_inserted(state):
    """螺丝已插入螺孔，仅需拧紧"""
    if (state.holding == 'screw'
            and state.screw_inserted is not None
            and state.current_screw is not None):
        return [('fasten_screw', state.current_screw, state.screw_inserted)]
    return False


def continue_screw_aligned(state):
    """螺丝已对准螺孔，需插入后拧紧"""
    if (state.holding == 'screw'
            and state.screw_aligned is not None
            and state.screw_inserted is None
            and state.current_screw is not None):
        return [
            ('insert_screw', state.current_screw, state.screw_aligned),
            ('fasten_screw', state.current_screw, state.screw_aligned)
        ]
    return False


def continue_screw_in_hand(state):
    """
    手持螺丝但未对准任何螺孔，自动分配到下一个待紧固螺孔
    按照 A→B→C→D 有序、E/F/G 随后的优先级
    """
    if (state.holding == 'screw'
            and state.screw_aligned is None
            and state.placement_ok == True):
        ordered_holes = ['hole_A', 'hole_B', 'hole_C', 'hole_D']
        random_holes = ['hole_E', 'hole_F', 'hole_G']

        for hole in ordered_holes + random_holes:
            if hole not in state.screws_tightened:
                screw = 'screw_' + hole.split('_')[1]
                return [
                    ('locating_screw', screw, hole),
                    ('insert_screw', screw, hole),
                    ('fasten_screw', screw, hole)
                ]
        return False
    return False


htn.declare_methods('continue_current_screw',
                    continue_screw_inserted,
                    continue_screw_aligned,
                    continue_screw_in_hand)


# ============================================================================
# 螺丝紧固任务 - Screw Fastening Tasks
# ============================================================================

def fasten_all_screws_method(state):
    """
    紧固所有7颗螺丝

    紧固顺序：
    - A → B → C → D 按顺序依次紧固
    - E、F、G 随机顺序紧固

    前置条件：放置正确且手中未持有螺丝（mid-screw 由顶层方法处理）
    """
    if state.placement_ok == True and state.holding != 'screw':
        ordered_holes = ['hole_A', 'hole_B', 'hole_C', 'hole_D']
        random_holes = ['hole_E', 'hole_F', 'hole_G']

        remaining_ordered = [h for h in ordered_holes if h not in state.screws_tightened]
        remaining_random = [h for h in random_holes if h not in state.screws_tightened]

        if not remaining_ordered and not remaining_random:
            return []

        subtasks = []

        for hole in remaining_ordered:
            screw = 'screw_' + hole.split('_')[1]
            subtasks.append(('tighten_screw', screw, hole))

        random.shuffle(remaining_random)
        for hole in remaining_random:
            screw = 'screw_' + hole.split('_')[1]
            subtasks.append(('tighten_screw', screw, hole))

        return subtasks
    return False


htn.declare_methods('fasten_all_screws', fasten_all_screws_method)


def tighten_screw_already_done(state, screw, hole):
    """螺丝已紧固时跳过（空操作）"""
    if hole in state.screws_tightened:
        return []
    return False


def tighten_screw_method(state, screw, hole):
    """
    紧固单个螺丝的完整流程：取螺丝 → 对准 → 插入 → 拧紧
    """
    if (state.placement_ok == True
            and hole not in state.screws_tightened
            and state.holding != 'screw'):
        return [
            ('fetch_screw',),
            ('locating_screw', screw, hole),
            ('insert_screw', screw, hole),
            ('fasten_screw', screw, hole)
        ]
    return False


htn.declare_methods('tighten_screw', tighten_screw_already_done, tighten_screw_method)


# ============================================================================
# 辅助方法 - Helper Methods
# ============================================================================

def check_completion(state):
    """检查安装是否完成"""
    required_holes = ['hole_A', 'hole_B', 'hole_C', 'hole_D',
                      'hole_E', 'hole_F', 'hole_G']
    return all(hole in state.screws_tightened for hole in required_holes)

