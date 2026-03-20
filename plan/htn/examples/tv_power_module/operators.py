"""
电视机电源模块安装 - 基础操作定义
TV Power Module Installation - Basic Operators Definition

Author: HTN Planner
Description: 定义电源模块安装过程中的所有基础操作（原子动作）
"""

from robot_htn import htn


# ============================================================================
# 检查操作 - Inspection Operators
# ============================================================================

def inspect_power_com(state):
    """
    检查料箱中是否存在电源模块
    Check if power component exists in material box
    
    前置条件：
    - 机器人在初始位置或料箱位置
    
    效果：
    - 更新 power_com_available 状态
    """
    if state.robot_pos in ['initial', 'material_box']:
        # 检查后设置电源模块可用状态
        state.power_com_available = state.power_com_in_box
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
    - obj: 要拿取的物体（如 'power_com'）
    - location: 物体所在位置（如 'material_box'）
    
    前置条件：
    - 物体在指定位置
    - 机器人在该位置
    - 机器人手为空
    - 如果是电源模块，必须已检查且可用
    
    效果：
    - 物体位置变为 'hand'
    - 机器人持有该物体
    """
    if obj == 'power_com':
        # 电源模块需要先检查
        if not (state.inspected and state.power_com_available):
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
    - location: 目标位置（如 'TV_panel'）
    
    前置条件：
    - 机器人持有该物体
    - 机器人在目标位置
    - 定位已完成
    
    效果：
    - 物体位置变为目标位置
    - 机器人手为空
    - 放置完成标志设为True
    """
    if (state.holding == obj 
        and state.robot_pos == location
        and state.locating_done == True):
        
        state.obj_pos[obj] = location
        state.holding = False
        state.placement_done = True
        # 模拟放置成功率（实际使用时可通过传感器判断）
        state.placement_ok = True  # 假设放置成功
        return state
    else:
        return False


def repick(state, obj, location):
    """
    重新抓取物体（放置不正确时）
    Re-pick object when placement is incorrect
    
    参数：
    - obj: 要重新抓取的物体
    - location: 物体当前位置
    
    前置条件：
    - 物体在指定位置
    - 机器人在该位置
    - 放置完成但不正确
    - 机器人手为空
    
    效果：
    - 物体位置变为 'hand'
    - 机器人持有该物体
    - 重置定位和放置状态
    """
    if (state.obj_pos.get(obj) == location
        and state.robot_pos == location
        and state.placement_done == True
        and state.placement_ok == False
        and state.holding == False):
        
        state.obj_pos[obj] = 'hand'
        state.holding = obj
        state.locating_done = False
        state.placement_done = False
        return state
    else:
        return False


# ============================================================================
# 移动操作 - Movement Operators
# ============================================================================

def move(state, obj, destination):
    """
    将持有的物体移动到目标位置上方
    Move held object to above destination
    
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
    机器人移动到指定位置
    Move robot to specified location
    
    参数：
    - destination: 目标位置
    
    效果：
    - 机器人位置变为目标位置
    """
    state.robot_pos = destination
    return state


# ============================================================================
# 定位操作 - Locating Operators
# ============================================================================

def locating(state, obj, target):
    """
    将物体与目标位置对准定位
    Align object with target location
    
    参数：
    - obj: 要定位的物体
    - target: 目标位置
    
    前置条件：
    - 机器人持有该物体
    - 机器人在目标位置
    
    效果：
    - 定位完成标志设为True
    """
    if (state.holding == obj 
        and state.robot_pos == target):
        
        state.locating_done = True
        return state
    else:
        return False


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
    - screw: 螺丝标识（如 'screw_A'）
    - hole: 螺孔标识（如 'hole_A'）
    
    前置条件：
    - 机器人持有螺丝
    - 电源模块已正确放置
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
    - 螺丝紧固完成
    - 机器人手为空
    - 更新已紧固螺丝列表
    """
    if state.screw_inserted == hole:
        state.screws_tightened.append(hole)
        state.holding = False
        state.screw_aligned = None
        state.screw_inserted = None
        state.current_screw = None
        return state
    else:
        return False


# ============================================================================
# 注册所有操作到HTN系统
# Register all operators to HTN system
# ============================================================================

htn.declare_operators(
    inspect_power_com,
    pick,
    place,
    repick,
    move,
    moveto,
    locating,
    fetch_screw,
    locating_screw,
    insert_screw,
    fasten_screw
)

