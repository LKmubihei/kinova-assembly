"""
洗衣机显示面板安装 - HTN规划主程序
Washing Machine Display Board Installation - HTN Planning Main Program

Author: HTN Planner
Description: 使用分层任务网络规划洗衣机显示面板的安装任务

场景描述：
1. 机器人通过 inspect_conveyor() 定位传送带上是否有洗衣机显示面板组件
2. 机器人 pick(wm_board, conveyor) 从传送带抓取显示面板组件
3. 机器人将显示面板组件移动到工作台上 move(wm_board, work_platform)
4. 机器人进行显示面板组件放置到工作台上 place(wm_board, work_platform)
5. 放置完成后，进行螺丝紧固，紧固顺序随机，一共9个螺丝
6. 螺丝紧固基础操作：fetch_screw, locating_screw, insert_screw, fasten_screw
7. 螺丝拧紧之后，将面板放回传送带：repick, move, place

预设场景支持：
- 从头开始        完整流程
- 检查完毕        从拿取开始
- 已拿取面板      从搬运到工作台开始
- 面板已放置      从螺丝紧固开始
- 部分螺丝已紧固  继续紧固剩余螺丝
- 螺丝全部紧固    归还面板到传送带
"""

from __future__ import print_function
import os
from pathlib import Path
import sys


def get_project_root():
    """返回工程根目录（通过标记文件 README.md 确认）"""
    current_dir = Path(__file__).resolve()
    for parent in current_dir.parents:
        if (parent / "README.md").exists():
            return parent
    raise FileNotFoundError("未找到工程根目录")


project_root = get_project_root()

abs_path = os.path.abspath(project_root)
if abs_path not in sys.path:
    sys.path.insert(0, abs_path)

from robot_htn import htn, State
from robot_htn.helpers import print_operators, print_methods, print_state

import examples.wm_display_board.operators as operators
import examples.wm_display_board.methods as methods
from examples.wm_display_board.state_configurator import PRESETS


def create_state_from_preset(preset_key):
    """
    根据预设场景创建初始状态和任务列表

    参数：
    - preset_key: 预设场景标识符

    返回：
    - (state, tasks): 初始状态和任务列表的元组
    """
    if preset_key not in PRESETS:
        raise ValueError(f"未知的预设场景: {preset_key}")

    preset = PRESETS[preset_key]
    st = preset["state"]

    state = State('wm_display_board_installation')

    # 机器人状态
    state.robot_pos = st["robot_pos"]
    board_pos = st["board_pos"]
    state.obj_pos = {'wm_board': board_pos}
    holding_val = st["holding"]
    state.holding = False if holding_val == "none" else holding_val

    # 传送带检查状态
    state.board_on_conveyor = st["board_on_conveyor"]
    state.board_available = st["board_available"]
    state.inspected = st["inspected"]

    # 面板放置状态
    state.placement_done = st["placement_done"]
    state.placement_ok = st["placement_ok"]

    # 螺丝紧固状态
    state.screws_available = st["screws_available"]
    state.screws_tightened = list(st["screws_tightened"])  # 深拷贝
    state.screw_aligned = None
    state.screw_inserted = None
    state.current_screw = None
    state.all_screws_done = st["all_screws_done"]

    # 归还状态
    state.board_returned = st["board_returned"]

    # 解析任务
    tasks = eval(preset["tasks"])

    return state, tasks


def create_initial_state():
    """
    创建默认初始状态（从头开始）

    状态变量说明：
    - robot_pos: 机器人当前位置 ('initial' / 'conveyor' / 'work_platform')
    - obj_pos: 各物体的位置字典 {'wm_board': 'conveyor'/'hand'/'work_platform'}
    - holding: 机器人当前持有的物体 (False / 'wm_board' / 'screw')
    - board_on_conveyor: 传送带上是否有显示面板组件
    - board_available: 显示面板组件是否可用（检查后更新）
    - inspected: 是否已完成传送带检查
    - placement_done: 面板是否已放置到工作台
    - placement_ok: 面板放置是否正确
    - screws_available: 可用螺丝数量（9颗）
    - screws_tightened: 已紧固的螺孔列表
    - screw_aligned: 当前对准的螺孔 (None / 'hole_x')
    - screw_inserted: 当前已插入螺丝的螺孔 (None / 'hole_x')
    - current_screw: 当前正在操作的螺丝 (None / 'screw_x')
    - all_screws_done: 所有9颗螺丝是否已紧固完成
    - board_returned: 面板是否已归还到传送带
    """
    state, tasks = create_state_from_preset("full_process")
    return state


def show_preset_menu():
    """
    显示预设场景选择菜单

    返回：
    - preset_key: 用户选择的预设场景标识符
    """
    print("\n" + "=" * 60)
    print("请选择预设场景 (Select Preset Scenario)")
    print("=" * 60)

    preset_items = list(PRESETS.items())
    for i, (key, preset) in enumerate(preset_items, 1):
        done_count = len(preset["state"]["screws_tightened"])
        status_bar = "█" * done_count + "░" * (9 - done_count)

        print(f"\n  [{i}] {preset['label']}")
        print(f"      {preset['desc']}")
        print(f"      螺丝进度: [{status_bar}] {done_count}/9")

    print(f"\n  [0] 启动可视化状态配置器 (GUI)")
    print("\n" + "-" * 60)

    while True:
        try:
            choice_str = input("请输入选项编号 (Enter option number) [1]: ").strip()
            if choice_str == '':
                choice = 1
            else:
                choice = int(choice_str)

            if choice == 0:
                return '_gui_configurator_'
            if 1 <= choice <= len(preset_items):
                selected_key = preset_items[choice - 1][0]
                selected_preset = PRESETS[selected_key]
                print(f"\n  ✓ 已选择: {selected_preset['label']}")
                return selected_key
            else:
                print(f"  请输入 0 ~ {len(preset_items)} 之间的数字")
        except ValueError:
            print("  请输入有效数字")
        except EOFError:
            return "full_process"


def print_plan_details(plan):
    """打印规划结果的详细信息"""
    print("\n" + "=" * 60)
    print("规划结果 (Plan Result)")
    print("=" * 60)

    if not plan:
        print("规划失败：无法找到可行的计划")
        return

    categories = {
        '检查操作': ['inspect_conveyor'],
        '拿取操作': ['pick', 'repick'],
        '移动操作': ['move', 'moveto'],
        '放置操作': ['place'],
        '螺丝操作': ['fetch_screw', 'locating_screw', 'insert_screw', 'fasten_screw']
    }

    print(f"\n总步骤数: {len(plan)}")
    print("-" * 60)

    for i, action in enumerate(plan, 1):
        action_name = action[0]
        action_args = action[1:] if len(action) > 1 else ()

        category = "其他操作"
        for cat, ops in categories.items():
            if action_name in ops:
                category = cat
                break

        if action_args:
            args_str = ", ".join(str(arg) for arg in action_args)
            print(f"  步骤 {i:2d} [{category}]: {action_name}({args_str})")
        else:
            print(f"  步骤 {i:2d} [{category}]: {action_name}()")

    print("-" * 60)

    # 操作统计
    print("\n操作统计:")
    for cat, ops in categories.items():
        count = sum(1 for action in plan if action[0] in ops)
        if count > 0:
            print(f"  - {cat}: {count} 次")

    # 螺丝紧固详情
    screw_actions = [a for a in plan if a[0] in ['locating_screw', 'insert_screw', 'fasten_screw']]
    if screw_actions:
        print("\n螺丝紧固顺序:")
        fastened = [a for a in plan if a[0] == 'fasten_screw']
        for i, action in enumerate(fastened, 1):
            print(f"  第{i}颗: {action[1]} → {action[2]}")


def launch_gui_with_config():
    """启动GUI并加载洗衣机显示面板配置"""
    import tkinter as tk
    from robot_htn.gui import HTNPlannerGUI

    config_path = os.path.join(
        project_root, 'configs', 'examples', 'wm_display_board.json'
    )

    root = tk.Tk()

    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    app = HTNPlannerGUI(root)

    if os.path.exists(config_path):
        app.load_config(config_path)
        app.is_modified = False
        app.update_title()
    else:
        print(f"警告: 配置文件不存在 {config_path}")

    root.mainloop()


def launch_state_configurator():
    """启动可视化状态配置器对话框，返回用户配置的状态和任务"""
    import tkinter as tk
    from examples.wm_display_board.state_configurator import WMStateConfiguratorDialog
    import json

    result = {'state_json': None, 'tasks_str': None}

    def on_apply(state_json, tasks_str):
        result['state_json'] = state_json
        result['tasks_str'] = tasks_str

    root = tk.Tk()
    root.title("洗衣机显示面板 - 状态配置")
    # 将根窗口设为最小不可见，避免 Toplevel 的 transient/grab_set 阻塞
    root.geometry("1x1+-100+-100")
    root.overrideredirect(True)

    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    dialog = WMStateConfiguratorDialog(root, on_apply=on_apply)
    # 解除 transient 和 grab，让对话框能独立显示
    dialog.transient("")
    dialog.grab_release()
    dialog.lift()
    dialog.focus_force()

    root.wait_window(dialog)
    root.destroy()

    if result['state_json'] and result['tasks_str']:
        state_dict = json.loads(result['state_json'])

        state = State(state_dict.get('name', 'wm_display_board_installation'))

        # 映射 JSON 字段到 State 对象
        state.robot_pos = state_dict['robot_pos']
        state.obj_pos = state_dict['obj_pos']
        holding = state_dict['holding']
        state.holding = False if holding is False or holding == "none" else holding
        state.board_on_conveyor = state_dict['board_on_conveyor']
        state.board_available = state_dict['board_available']
        state.inspected = state_dict['inspected']
        state.placement_done = state_dict['placement_done']
        state.placement_ok = state_dict['placement_ok']
        state.screws_available = state_dict['screws_available']
        state.screws_tightened = list(state_dict['screws_tightened'])
        state.screw_aligned = state_dict.get('screw_aligned')
        state.screw_inserted = state_dict.get('screw_inserted')
        state.current_screw = state_dict.get('current_screw')
        state.all_screws_done = state_dict.get('all_screws_done', False)
        state.board_returned = state_dict.get('board_returned', False)

        tasks = eval(result['tasks_str'])
        return state, tasks

    return None, None


def run_planning(state, tasks, preset_label="自定义"):
    """
    执行HTN规划并输出结果

    参数：
    - state: 初始状态
    - tasks: 任务列表
    - preset_label: 场景名称（用于输出显示）
    """
    # 显示初始状态
    print("\n" + "=" * 60)
    print(f"初始状态 (Initial State) — {preset_label}")
    print("=" * 60)
    print_state(state)

    # 显示任务目标
    print("\n" + "=" * 60)
    print("任务目标 (Task Goal)")
    print("=" * 60)
    print(f"  任务列表: {tasks}")

    # 开始规划
    print("\n" + "=" * 60)
    print("开始规划 (Starting Planning)")
    print("=" * 60)

    result = htn.plan(
        state,
        tasks,
        htn.get_operators(),
        htn.get_methods(),
        verbose=1
    )

    print_plan_details(result)

    return result


def main():
    """主函数"""
    print("\n" + "=" * 60)
    print("洗衣机显示面板安装 - HTN规划系统")
    print("Washing Machine Display Board Installation - HTN Planning System")
    print("=" * 60)

    # 显示已注册的操作
    print("\n已注册的操作 (Registered Operators):")
    print("-" * 40)
    print_operators(htn.get_operators())

    # 显示已注册的方法
    print("\n已注册的方法 (Registered Methods):")
    print("-" * 40)
    print_methods(htn.get_methods())

    # 选择预设场景
    preset_key = show_preset_menu()

    if preset_key == '_gui_configurator_':
        # 启动可视化配置器
        print("\n启动可视化状态配置器...")
        state, tasks = launch_state_configurator()
        if state is None:
            print("  用户取消了配置，使用默认场景（从头开始）")
            state, tasks = create_state_from_preset("full_process")
            preset_label = PRESETS["full_process"]["label"]
        else:
            preset_label = "自定义配置"
    else:
        state, tasks = create_state_from_preset(preset_key)
        preset_label = PRESETS[preset_key]["label"]

    # 执行规划
    result = run_planning(state, tasks, preset_label)

    # 可视化选项
    print("\n" + "=" * 60)
    print("后续操作:")
    print("  [1] 查看任务网络可视化 (GUI)")
    print("  [2] 选择其他预设场景重新规划")
    print("  [其他] 退出")
    print("-" * 60)

    try:
        response = input("请选择: ").strip()
        if response == '1':
            print("启动可视化界面...")
            launch_gui_with_config()
        elif response == '2':
            # 递归调用，重新选择场景
            main()
    except EOFError:
        print("(退出)")

    return result


if __name__ == '__main__':
    main()
