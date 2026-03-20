"""
洗衣机显示面板安装 - 任务分解方法定义
Washing Machine Display Board Installation - Task Decomposition Methods

Author: HTN Planner
Description: 定义洗衣机显示面板安装的分层任务网络方法

任务层次结构：
├── install_wm_display_board (顶层任务)
│   ├── inspect_and_pickup (检查并拿取面板)
│   │   ├── moveto(conveyor)
│   │   ├── inspect_conveyor()
│   │   └── pick(wm_board, conveyor)
│   ├── transport_and_place (搬运并放置面板到工作台)
│   │   ├── move(wm_board, work_platform)
│   │   └── place(wm_board, work_platform)
│   ├── fasten_all_screws (紧固所有9颗螺丝)
│   │   └── tighten_screw(screw_i, hole_i) × 9 (随机顺序)
│   │       ├── fetch_screw()
│   │       ├── locating_screw(screw_i, hole_i)
│   │       ├── insert_screw(screw_i, hole_i)
│   │       └── fasten_screw(screw_i, hole_i)
│   └── return_to_conveyor (归还面板到传送带)
│       ├── repick(wm_board, work_platform)
│       ├── move(wm_board, conveyor)
│       └── place(wm_board, conveyor)
"""

from robot_htn import htn
import random


# ============================================================================
# 顶层任务 - Top Level Task
# ============================================================================

def install_wm_display_board(state, board):
    """
    安装洗衣机显示面板的顶层任务
    Top-level task for washing machine display board installation

    任务分解（根据当前状态自动选择起点）：
    1. 检查传送带并拿取面板
    2. 搬运面板到工作台并放置
    3. 紧固所有9颗螺丝
    4. 将面板归还到传送带

    参数：
    - board: 面板组件标识（'wm_board'）

    支持从任意中间状态恢复规划（完全断点续做），包括：
    - 面板在传送带上（未检查 / 已检查）
    - 机器人已持有面板
    - 面板已放置在工作台上
    - 螺丝操作中途（fetch/locate/insert 之后）
    - 螺丝已全部紧固，待归还
    - 归还途中（repick/move 之后）
    """
    if state.board_returned:
        return []

    if not state.all_screws_done:
        # 面板在传送带上
        if state.obj_pos.get(board) == 'conveyor' and state.holding == False:
            return [
                ('inspect_and_pickup', board, 'conveyor'),
                ('transport_and_place', board, 'work_platform'),
                ('fasten_all_screws',),
                ('return_to_conveyor', board)
            ]
        # 机器人已持有面板，从搬运到工作台开始
        elif state.holding == board:
            return [
                ('transport_and_place', board, 'work_platform'),
                ('fasten_all_screws',),
                ('return_to_conveyor', board)
            ]
        # 面板已在工作台上且放置正确，继续紧固螺丝
        elif state.obj_pos.get(board) == 'work_platform' and state.placement_ok:
            if state.holding == 'screw':
                # ── 断点恢复：螺丝操作中途（fetch/locate/insert 之后）──
                subtasks = _resume_current_screw(state)
                subtasks.append(('fasten_all_screws',))
                subtasks.append(('return_to_conveyor', board))
                return subtasks
            else:
                return [
                    ('fasten_all_screws',),
                    ('return_to_conveyor', board)
                ]
    else:
        # 螺丝已全部紧固 → 归还（支持 repick/move 之后的断点续做）
        return [('return_to_conveyor', board)]

    return False


def _resume_current_screw(state):
    """
    从螺丝操作中途恢复：根据 screw_inserted / screw_aligned / 无对准
    三种子状态，生成剩余原子操作序列。
    """
    if state.screw_inserted and state.current_screw:
        # insert 之后 → 只需 fasten
        return [('fasten_screw', state.current_screw, state.screw_inserted)]
    elif state.screw_aligned and state.current_screw:
        # locate 之后 → insert + fasten
        return [
            ('insert_screw', state.current_screw, state.screw_aligned),
            ('fasten_screw', state.current_screw, state.screw_aligned)
        ]
    else:
        # fetch 之后，螺丝在手但未对准 → 选第一个待紧固孔位
        remaining = [f'hole_{i}' for i in range(1, 10)
                     if f'hole_{i}' not in state.screws_tightened]
        if remaining:
            hole = remaining[0]
            screw = 'screw_' + hole.split('_')[1]
            return [
                ('locating_screw', screw, hole),
                ('insert_screw', screw, hole),
                ('fasten_screw', screw, hole)
            ]
        return []


htn.declare_methods('install_wm_display_board', install_wm_display_board)


# ============================================================================
# 检查和拿取任务 - Inspect and Pickup Task
# ============================================================================

def inspect_and_pickup_method(state, board, location):
    """
    检查传送带并从传送带上拿取显示面板
    Inspect conveyor and pickup display board

    任务分解：
    1. 移动到传送带位置（如果不在）
    2. 检查传送带
    3. 拿取面板
    """
    if state.holding == False and state.obj_pos.get(board) == location:
        subtasks = []

        # 如果不在传送带位置，先移动过去
        if state.robot_pos != location:
            subtasks.append(('moveto', location))

        # 如果还没检查，先检查
        if not state.inspected:
            subtasks.append(('inspect_conveyor',))

        # 拿取面板
        subtasks.append(('pick', board, location))

        return subtasks
    else:
        return False


htn.declare_methods('inspect_and_pickup', inspect_and_pickup_method)


# ============================================================================
# 搬运和放置任务 - Transport and Place Task
# ============================================================================

def transport_and_place_method(state, board, destination):
    """
    将面板从当前位置搬运到工作台并放置
    Transport board to work platform and place it

    任务分解：
    1. 移动面板到工作台
    2. 放置面板到工作台
    """
    if state.holding == board:
        return [
            ('move', board, destination),
            ('place', board, destination)
        ]
    else:
        return False


htn.declare_methods('transport_and_place', transport_and_place_method)


# ============================================================================
# 螺丝紧固任务 - Screw Fastening Tasks
# ============================================================================

def fasten_all_screws_method(state):
    """
    紧固所有9颗螺丝（随机顺序）
    Fasten all 9 screws in random order

    螺丝编号：screw_1 ~ screw_9
    螺孔编号：hole_1 ~ hole_9
    紧固顺序：完全随机
    """
    if state.placement_ok == True:
        all_holes = [f'hole_{i}' for i in range(1, 10)]

        remaining = [h for h in all_holes if h not in state.screws_tightened]

        if not remaining:
            return []  # 所有螺丝已紧固

        # 随机打乱紧固顺序
        random.shuffle(remaining)

        subtasks = []
        for hole in remaining:
            screw = 'screw_' + hole.split('_')[1]
            subtasks.append(('tighten_screw', screw, hole))

        return subtasks
    else:
        return False


htn.declare_methods('fasten_all_screws', fasten_all_screws_method)


def tighten_screw_already_done(state, screw, hole):
    """
    螺丝已紧固时跳过（空操作）
    Skip if screw is already tightened (no-op)
    """
    if hole in state.screws_tightened:
        return []
    return False


def tighten_screw_method(state, screw, hole):
    """
    紧固单个螺丝的完整流程
    Complete process for tightening a single screw

    任务分解：
    1. fetch_screw()      - 取螺丝
    2. locating_screw()   - 将螺丝与螺孔对准
    3. insert_screw()     - 将螺丝插入螺孔
    4. fasten_screw()     - 将螺丝拧紧
    """
    if (state.placement_ok == True
        and hole not in state.screws_tightened):
        return [
            ('fetch_screw',),
            ('locating_screw', screw, hole),
            ('insert_screw', screw, hole),
            ('fasten_screw', screw, hole)
        ]
    else:
        return False


htn.declare_methods('tighten_screw', tighten_screw_already_done, tighten_screw_method)


# ============================================================================
# 归还任务 - Return to Conveyor Task
# ============================================================================

def return_to_conveyor_method(state, board):
    """
    将面板归还到传送带（从工作台开始）
    Return display board to conveyor - starting from work platform

    前置条件：
    - 所有螺丝已紧固完成
    - 面板在工作台上
    - 机器人手为空
    """
    if (state.all_screws_done == True
        and state.obj_pos.get(board) == 'work_platform'
        and state.holding == False):
        return [
            ('repick', board, 'work_platform'),
            ('move', board, 'conveyor'),
            ('place', board, 'conveyor')
        ]
    else:
        return False


def return_to_conveyor_holding(state, board):
    """
    断点恢复：归还途中已持有面板（repick/move 之后）
    Resume return - already holding the board after repick or move

    根据机器人位置决定剩余步骤：
    - 还在工作台 → move + place
    - 已在传送带 → place
    """
    if (state.all_screws_done == True
        and state.holding == board):
        subtasks = []
        if state.robot_pos != 'conveyor':
            subtasks.append(('move', board, 'conveyor'))
        subtasks.append(('place', board, 'conveyor'))
        return subtasks
    else:
        return False


htn.declare_methods('return_to_conveyor',
                    return_to_conveyor_holding,
                    return_to_conveyor_method)


# ============================================================================
# 辅助方法 - Helper Methods
# ============================================================================

def check_completion(state):
    """
    检查安装是否完成
    Check if the entire installation process is complete

    完成条件：
    - 所有9颗螺丝已紧固
    - 面板已归还到传送带
    """
    required_holes = [f'hole_{i}' for i in range(1, 10)]
    screws_done = all(hole in state.screws_tightened for hole in required_holes)
    return screws_done and state.board_returned

