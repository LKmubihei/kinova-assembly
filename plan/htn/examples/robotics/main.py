"""
Robotic task plan test data for robot_htn 1.1.
Author: guangxi wan 

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

# 将目标路径添加到sys.path
abs_path = os.path.abspath(project_root)
if abs_path not in sys.path:
    sys.path.insert(0, abs_path)  # 优先搜索该路径

from robot_htn import htn, State
from robot_htn.helpers import print_operators, print_methods, print_state
import examples.robotics.operators as operators
import examples.robotics.methods as methods

print('')
print_operators(htn.get_operators())
print('')
print_methods(htn.get_methods())

#############     beginning of tests     ################

print("""
****************************************
First, test robot_htn on some of the operators and smaller tasks
****************************************
""")

"""
A state is a collection of all of the state variables and their values. Every
state variable in the domain should have a value.
"""

# 使用新的State类构造函数
state1 = State('state1')
state1.pos = {'regulator':'table', 'robot':'inital'}
state1.clear = {'regulator':True, "table":True, "placement_regulator":True}
state1.holding = False

print_state(state1)
print('')

# 执行规划并打印结果
result = htn.plan(state1, [('assemble','regulator','placement_regulator')], 
                  htn.get_operators(), htn.get_methods(), verbose=1)

if result:
    print("\n规划结果:")
    for i, action in enumerate(result):
        print(f"  步骤 {i+1}: {action}")
else:
    print("\n规划失败：无法找到可行的计划")

# 可选：启动GUI查看可视化
print("\n是否要查看任务网络可视化? (y/n): ", end='')
response = input().strip().lower()
if response == 'y':
    from robot_htn.gui import main as gui_main
    print("启动可视化界面...")
    gui_main()








