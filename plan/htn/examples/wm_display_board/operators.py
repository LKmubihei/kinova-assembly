"""
洗衣机显示面板安装 - 基础操作定义
Washing Machine Display Board Installation - Basic Operators Definition

Author: HTN Planner
Description: 定义洗衣机显示面板安装过程中的所有基础操作（原子动作）

场景说明：
- 传送带(conveyor): 面板组件初始所在位置，也是最终归还位置
- 工作台(work_platform): 螺丝紧固的工作位置
- 9个螺丝(screw_1~screw_9)，对应9个螺孔(hole_1~hole_9)
"""

from robot_htn import htn


# ============================================================================
# 检查操作 - Inspection Operators
# ============================================================================

def inspect_conveyor(state):
    """
    检查传送带上是否存在洗衣机显示面板组件
    Check if washing machine display board exists on conveyor

    前置条件：
    - 机器人在初始位置或传送带位置

    效果：
    - 更新 board_available 状态
    - 标记检查已完成
    """
    if state.robot_pos in ['initial', 'conveyor']:
        state.board_available = state.board_on_conveyor
        state.inspected = True
        return state
    else:
        return False


# ============================================================================
# 拿取和放置操作 - Pick and Place Operators
# ============================================================================

def pick(state, obj, location):
    """
    从指定位置拿取物体
    Pick up object from specified location

    参数：
    - obj: 要拿取的物体（如 'wm_board'）
    - location: 物体所在位置（如 'conveyor'）

    前置条件：
    - 物体在指定位置
    - 机器人在该位置
    - 机器人手为空
    - 如果是显示面板，必须已检查且可用

    效果：
    - 物体位置变为 'hand'
    - 机器人持有该物体
    """
    if obj == 'wm_board':
        if not (state.inspected and state.board_available):
            return False

    if (state.obj_pos.get(obj) == location
        and state.robot_pos == location
        and state.holding == False):

        state.obj_pos[obj] = 'hand'
        state.holding = obj
        return state
    else:
        return False


def place(state, obj, location):
    """
    将物体放置到指定位置
    Place object at specified location

    参数：
    - obj: 要放置的物体
    - location: 目标位置（如 'work_platform' 或 'conveyor'）

    前置条件：
    - 机器人持有该物体
    - 机器人在目标位置

    效果：
    - 物体位置变为目标位置
    - 机器人手为空
    - 如果放置到工作台，标记放置完成
    - 如果放置到传送带（归还），标记归还完成
    """
    if (state.holding == obj
        and state.robot_pos == location):

        state.obj_pos[obj] = location
        state.holding = False

        if location == 'work_platform':
            state.placement_done = True
            state.placement_ok = True

        if location == 'conveyor':
            state.board_returned = True

        return state
    else:
        return False


def repick(state, obj, location):
    """
    从工作台重新抓取物体（螺丝紧固完成后）
    Re-pick object from work platform after screw fastening is done

    参数：
    - obj: 要重新抓取的物体
    - location: 物体当前位置（work_platform）

    前置条件：
    - 物体在指定位置
    - 机器人在该位置
    - 所有螺丝已紧固完成
    - 机器人手为空

    效果：
    - 物体位置变为 'hand'
    - 机器人持有该物体
    """
    if (state.obj_pos.get(obj) == location
        and state.robot_pos == location
        and state.all_screws_done == True
        and state.holding == False):

        state.obj_pos[obj] = 'hand'
        state.holding = obj
        return state
    else:
        return False


# ============================================================================
# 移动操作 - Movement Operators
# ============================================================================

def move(state, obj, destination):
    """
    将持有的物体移动到目标位置
    Move held object to destination

    参数：
    - obj: 正在持有的物体
    - destination: 目标位置

    前置条件：
    - 机器人持有该物体

    效果：
    - 机器人位置变为目标位置
    """
    if state.holding == obj:
        state.robot_pos = destination
        return state
    else:
        return False


def moveto(state, destination):
    """
    机器人移动到指定位置（空手）
    Move robot to specified location (empty-handed)

    参数：
    - destination: 目标位置

    效果：
    - 机器人位置变为目标位置
    """
    state.robot_pos = destination
    return state


# ============================================================================
# 螺丝操作 - Screw Operators
# ============================================================================

def fetch_screw(state):
    """
    取一颗螺丝
    Fetch a screw

    前置条件：
    - 机器人手为空
    - 有可用螺丝

    效果：
    - 机器人持有一颗螺丝
    - 可用螺丝数量减1
    """
    if state.holding == False and state.screws_available > 0:
        state.holding = 'screw'
        state.screws_available -= 1
        return state
    else:
        return False


def locating_screw(state, screw, hole):
    """
    将螺丝与螺孔对准
    Align screw with screw hole

    参数：
    - screw: 螺丝标识（如 'screw_1'）
    - hole: 螺孔标识（如 'hole_1'）

    前置条件：
    - 机器人持有螺丝
    - 显示面板已正确放置在工作台上
    - 该螺孔尚未紧固

    效果：
    - 螺丝已对准指定螺孔
    """
    if (state.holding == 'screw'
        and state.placement_ok == True
        and hole not in state.screws_tightened):

        state.screw_aligned = hole
        state.current_screw = screw
        return state
    else:
        return False


def insert_screw(state, screw, hole):
    """
    将螺丝插入到螺孔里
    Insert screw into hole

    参数：
    - screw: 螺丝标识
    - hole: 螺孔标识

    前置条件：
    - 螺丝已对准该螺孔
    - 当前螺丝匹配

    效果：
    - 螺丝已插入
    """
    if (state.screw_aligned == hole
        and state.current_screw == screw):

        state.screw_inserted = hole
        return state
    else:
        return False


def fasten_screw(state, screw, hole):
    """
    将螺丝拧紧
    Fasten/tighten the screw

    参数：
    - screw: 螺丝标识
    - hole: 螺孔标识

    前置条件：
    - 螺丝已插入该螺孔

    效果：
    - 螺丝紧固完成，加入已紧固列表
    - 机器人手为空
    - 重置螺丝操作中间状态
    """
    if state.screw_inserted == hole:
        state.screws_tightened.append(hole)
        state.holding = False
        state.screw_aligned = None
        state.screw_inserted = None
        state.current_screw = None

        # 检查是否所有9颗螺丝都已紧固
        required_holes = [f'hole_{i}' for i in range(1, 10)]
        if all(h in state.screws_tightened for h in required_holes):
            state.all_screws_done = True

        return state
    else:
        return False


# ============================================================================
# 注册所有操作到HTN系统
# Register all operators to HTN system
# ============================================================================

htn.declare_operators(
    inspect_conveyor,
    pick,
    place,
    repick,
    move,
    moveto,
    fetch_screw,
    locating_screw,
    insert_screw,
    fasten_screw
)

