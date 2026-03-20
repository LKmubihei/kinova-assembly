"""
电视机电源模块安装 - HTN规划主程序
TV Power Module Installation - HTN Planning Main Program

Author: HTN Planner
Description: 使用分层任务网络规划电视机电源模块的安装任务

场景描述：
1. 机器人检查料箱中是否存在电源模块 (inspect_power_com)
2. 机器人从料箱中拿取电源模块 (pick)
3. 机器人将电源模块移动到电视机面板上方 (move)
4. 机器人进行电源模块与安装位置的定位 (locating)
5. 机器人放置电源模块 (place)
6. 如果放置不对，进行重新抓取和二次定位 (repick)
7. 紧固螺丝A、B、C、D（A先紧固，其余随机）
8. 螺丝紧固基础操作：fetch_screw, locating_screw, insert_screw, fasten_screw
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

import examples.tv_power_module.operators as operators
import examples.tv_power_module.methods as methods


def create_initial_state():
    """
    创建初始状态
    
    状态变量说明：
    - robot_pos: 机器人当前位置
    - obj_pos: 各物体的位置字典
    - holding: 机器人当前持有的物体（False表示空手）
    - power_com_in_box: 料箱中是否有电源模块
    - power_com_available: 电源模块是否可用（检查后更新）
    - inspected: 是否已完成检查
    - locating_done: 定位是否完成
    - placement_done: 放置是否完成
    - placement_ok: 放置是否正确
    - screws_available: 可用螺丝数量
    - screws_tightened: 已紧固的螺孔列表
    - screw_aligned / screw_inserted / current_screw: 螺丝操作中间状态
    - all_screws_done: 所有螺丝是否紧固完成
    """
    state = State('tv_power_module_installation')
    
    state.robot_pos = 'initial'
    state.obj_pos = {'power_com': 'material_box'}
    state.holding = False
    
    state.power_com_in_box = True
    state.power_com_available = False
    state.inspected = False
    
    state.locating_done = False
    state.placement_done = False
    state.placement_ok = False
    
    state.screws_available = 10
    state.screws_tightened = []
    state.screw_aligned = None
    state.screw_inserted = None
    state.current_screw = None
    state.all_screws_done = False
    
    return state


def print_plan_details(plan):
    """打印规划结果的详细信息"""
    print("\n" + "=" * 60)
    print("规划结果 (Plan Result)")
    print("=" * 60)
    
    if not plan:
        print("规划失败：无法找到可行的计划")
        return
    
    categories = {
        '检查操作': ['inspect_power_com'],
        '拿取操作': ['pick', 'repick'],
        '移动操作': ['move', 'moveto'],
        '定位操作': ['locating', 'locating_screw'],
        '放置操作': ['place'],
        '螺丝操作': ['fetch_screw', 'insert_screw', 'fasten_screw']
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
    
    print("\n操作统计:")
    for cat, ops in categories.items():
        count = sum(1 for action in plan if action[0] in ops)
        if count > 0:
            print(f"  - {cat}: {count} 次")


def launch_gui_with_config():
    """启动GUI并加载电视机电源模块配置"""
    import tkinter as tk
    from robot_htn.gui import HTNPlannerGUI

    config_path = os.path.join(
        project_root, 'configs', 'examples', 'tv_power_module.json'
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


def main():
    """主函数"""
    print("\n" + "=" * 60)
    print("电视机电源模块安装 - HTN规划系统")
    print("TV Power Module Installation - HTN Planning System")
    print("=" * 60)

    print("\n已注册的操作 (Registered Operators):")
    print("-" * 40)
    print_operators(htn.get_operators())

    print("\n已注册的方法 (Registered Methods):")
    print("-" * 40)
    print_methods(htn.get_methods())

    print("\n" + "=" * 60)
    print("初始状态 (Initial State)")
    print("=" * 60)
    state = create_initial_state()
    print_state(state)

    print("\n" + "=" * 60)
    print("任务目标 (Task Goal)")
    print("=" * 60)
    print("  安装电源模块到电视机面板并紧固所有螺丝")
    print("  Install power component to TV panel and fasten all screws")

    print("\n" + "=" * 60)
    print("开始规划 (Starting Planning)")
    print("=" * 60)

    tasks = [('install_power_module', 'power_com', 'TV_panel')]

    result = htn.plan(
        state,
        tasks,
        htn.get_operators(),
        htn.get_methods(),
        verbose=1
    )

    print_plan_details(result)

    print("\n" + "=" * 60)
    print("是否要查看任务网络可视化? (y/n): ", end='')
    try:
        response = input().strip().lower()
        if response == 'y':
            print("启动可视化界面...")
            launch_gui_with_config()
    except EOFError:
        print("(跳过可视化)")

    return result


if __name__ == '__main__':
    main()
