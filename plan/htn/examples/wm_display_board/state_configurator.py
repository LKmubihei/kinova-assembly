"""
洗衣机显示面板安装 - 可视化状态配置器
Washing Machine Display Board Installation - Visual State Configurator

提供可视化界面配置不同初始状态，支持从任意阶段开始规划。
内置状态一致性自动校正和约束校验，避免配出无法规划的状态。

预设场景：
1. 从头开始        - 完整流程：检查传送带 → 拿取 → 搬运 → 放置 → 紧固 → 归还
2. 检查完毕        - 面板已检测到，从拿取开始
3. 已拿取面板      - 机器人持有面板，从搬运到工作台开始
4. 面板已放置      - 面板已在工作台上，从螺丝紧固开始
5. 部分螺丝已紧固  - 5颗螺丝已紧固，继续紧固剩余4颗
6. 螺丝全部紧固    - 9颗螺丝全部完成，准备归还传送带
"""

import tkinter as tk
from tkinter import ttk, messagebox
import json
import sys
import ast

from robot_htn import htn, State

try:
    from robot_htn.gui import AppleColors, AppleFonts, ScaleManager
except ImportError:
    class AppleColors:
        BG_PRIMARY = "#FFFFFF"; BG_SECONDARY = "#F5F5F7"; BG_TERTIARY = "#E8E8ED"
        TEXT_PRIMARY = "#1D1D1F"; TEXT_SECONDARY = "#86868B"; TEXT_TERTIARY = "#AEAEB2"
        TEXT_ON_ACCENT = "#FFFFFF"; ACCENT_BLUE = "#007AFF"; ACCENT_BLUE_HOVER = "#0056CC"
        ACCENT_BLUE_LIGHT = "#E3F2FF"; SUCCESS = "#34C759"; WARNING = "#FF9F0A"
        ERROR = "#FF3B30"; BORDER = "#D2D2D7"; SEPARATOR = "#C6C6C8"
        CARD_BG = "#FFFFFF"; INPUT_BG = "#FFFFFF"
    class AppleFonts:
        FAMILY = "Segoe UI" if sys.platform == "win32" else "SF Pro Display"
        TITLE = (FAMILY, 20, "bold"); HEADLINE = (FAMILY, 16, "bold")
        BODY = (FAMILY, 13); BODY_BOLD = (FAMILY, 13, "bold")
        CAPTION = (FAMILY, 11); CAPTION_BOLD = (FAMILY, 11, "bold")
    class ScaleManager:
        @classmethod
        def s(cls, v): return v


# ============================================================================
# 预设场景 - Preset Scenarios
# ============================================================================

PRESETS = {
    "full_process": {
        "label": "从头开始",
        "desc":  "完整流程：检查传送带 → 拿取面板 → 搬运到工作台 → 放置 → 紧固9颗螺丝 → 归还传送带",
        "state": {
            "robot_pos": "initial",
            "board_pos": "conveyor",
            "holding": "none",
            "board_on_conveyor": True,
            "board_available": False,
            "inspected": False,
            "placement_done": False,
            "placement_ok": False,
            "screws_available": 9,
            "screws_tightened": [],
            "all_screws_done": False,
            "board_returned": False,
        },
        "tasks": '[("install_wm_display_board", "wm_board")]',
    },
    "after_inspection": {
        "label": "检查完毕",
        "desc":  "传送带已检查，面板可用，从拿取开始",
        "state": {
            "robot_pos": "conveyor",
            "board_pos": "conveyor",
            "holding": "none",
            "board_on_conveyor": True,
            "board_available": True,
            "inspected": True,
            "placement_done": False,
            "placement_ok": False,
            "screws_available": 9,
            "screws_tightened": [],
            "all_screws_done": False,
            "board_returned": False,
        },
        "tasks": '[("install_wm_display_board", "wm_board")]',
    },
    "after_pickup": {
        "label": "已拿取面板",
        "desc":  "机器人已持有面板，从搬运到工作台开始",
        "state": {
            "robot_pos": "conveyor",
            "board_pos": "hand",
            "holding": "wm_board",
            "board_on_conveyor": True,
            "board_available": True,
            "inspected": True,
            "placement_done": False,
            "placement_ok": False,
            "screws_available": 9,
            "screws_tightened": [],
            "all_screws_done": False,
            "board_returned": False,
        },
        "tasks": '[("install_wm_display_board", "wm_board")]',
    },
    "after_placement": {
        "label": "面板已放置",
        "desc":  "面板已放置在工作台上，从螺丝紧固开始",
        "state": {
            "robot_pos": "work_platform",
            "board_pos": "work_platform",
            "holding": "none",
            "board_on_conveyor": True,
            "board_available": True,
            "inspected": True,
            "placement_done": True,
            "placement_ok": True,
            "screws_available": 9,
            "screws_tightened": [],
            "all_screws_done": False,
            "board_returned": False,
        },
        "tasks": '[("install_wm_display_board", "wm_board")]',
    },
    "partial_screws": {
        "label": "部分螺丝已紧固",
        "desc":  "螺丝1~5已紧固，继续紧固剩余4颗螺丝",
        "state": {
            "robot_pos": "work_platform",
            "board_pos": "work_platform",
            "holding": "none",
            "board_on_conveyor": True,
            "board_available": True,
            "inspected": True,
            "placement_done": True,
            "placement_ok": True,
            "screws_available": 4,
            "screws_tightened": ["hole_1", "hole_2", "hole_3", "hole_4", "hole_5"],
            "all_screws_done": False,
            "board_returned": False,
        },
        "tasks": '[("install_wm_display_board", "wm_board")]',
    },
    "screws_done": {
        "label": "螺丝全部紧固",
        "desc":  "9颗螺丝已全部紧固，准备将面板归还到传送带",
        "state": {
            "robot_pos": "work_platform",
            "board_pos": "work_platform",
            "holding": "none",
            "board_on_conveyor": True,
            "board_available": True,
            "inspected": True,
            "placement_done": True,
            "placement_ok": True,
            "screws_available": 0,
            "screws_tightened": [f"hole_{i}" for i in range(1, 10)],
            "all_screws_done": True,
            "board_returned": False,
        },
        "tasks": '[("install_wm_display_board", "wm_board")]',
    },
}


# ============================================================================
# State Configurator Dialog (GUI)
# ============================================================================

class WMStateConfiguratorDialog(tk.Toplevel):
    """洗衣机显示面板安装 — 可视化状态配置对话框（含状态一致性校验）"""

    TOTAL_SCREWS = 9  # 螺丝总数

    def __init__(self, parent, on_apply=None):
        super().__init__(parent)
        self.on_apply = on_apply
        self.title("状态配置 - 洗衣机显示面板安装")
        self.geometry("860x820")
        self.configure(bg=AppleColors.BG_PRIMARY)
        self.resizable(True, True)
        self.minsize(780, 720)

        self.transient(parent)
        self.grab_set()

        self._is_applying_preset = False
        self._is_enforcing = False
        self._prev_snapshot = None
        self._state_var_traces = []

        self._build_ui()
        self._bind_state_var_traces()
        self._apply_preset("full_process")

    # ================================================================== UI
    def _build_ui(self):
        s = ScaleManager.s

        outer = tk.Frame(self, bg=AppleColors.BG_PRIMARY)
        outer.pack(fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(outer, bg=AppleColors.BG_PRIMARY, highlightthickness=0)
        vbar = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        canvas.configure(yscrollcommand=vbar.set)
        vbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._container = tk.Frame(canvas, bg=AppleColors.BG_PRIMARY)
        canvas.create_window((0, 0), window=self._container, anchor="nw")
        self._container.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind_all(
            "<MouseWheel>",
            lambda e: canvas.yview_scroll(-1 * (e.delta // 120), "units"))

        pad = s(24)
        inner = tk.Frame(self._container, bg=AppleColors.BG_PRIMARY)
        inner.pack(fill=tk.BOTH, expand=True, padx=pad, pady=pad)

        # Title
        tk.Label(inner, text="洗衣机显示面板安装 — 状态配置",
                 font=AppleFonts.TITLE, bg=AppleColors.BG_PRIMARY,
                 fg=AppleColors.TEXT_PRIMARY).pack(anchor=tk.W, pady=(0, s(4)))
        tk.Label(inner,
                 text="选择预设场景或手动调整各状态变量，系统会自动校验一致性",
                 font=AppleFonts.CAPTION, bg=AppleColors.BG_PRIMARY,
                 fg=AppleColors.TEXT_SECONDARY).pack(anchor=tk.W, pady=(0, s(16)))

        self._build_preset_section(inner)

        tk.Frame(inner, bg=AppleColors.SEPARATOR, height=1).pack(
            fill=tk.X, pady=s(14))

        self._build_state_section(inner)

        tk.Frame(inner, bg=AppleColors.SEPARATOR, height=1).pack(
            fill=tk.X, pady=s(14))

        self._build_validation_section(inner)

        tk.Frame(inner, bg=AppleColors.SEPARATOR, height=1).pack(
            fill=tk.X, pady=s(14))

        self._build_task_section(inner)

        tk.Frame(inner, bg=AppleColors.SEPARATOR, height=1).pack(
            fill=tk.X, pady=s(14))

        self._build_preview_section(inner)
        self._build_action_buttons(inner)

    # -------------------------------------------------------------- Presets
    def _build_preset_section(self, parent):
        s = ScaleManager.s
        frame = tk.LabelFrame(parent, text="  预设场景  ",
                              font=AppleFonts.HEADLINE,
                              bg=AppleColors.BG_PRIMARY,
                              fg=AppleColors.TEXT_PRIMARY,
                              padx=s(12), pady=s(10))
        frame.pack(fill=tk.X, pady=(0, s(4)))

        self._preset_var = tk.StringVar(value="full_process")
        btn_frame = tk.Frame(frame, bg=AppleColors.BG_PRIMARY)
        btn_frame.pack(fill=tk.X)

        for i, (key, preset) in enumerate(PRESETS.items()):
            tk.Radiobutton(
                btn_frame, text=preset["label"], variable=self._preset_var,
                value=key, command=lambda k=key: self._apply_preset(k),
                font=AppleFonts.BODY_BOLD, bg=AppleColors.BG_PRIMARY,
                fg=AppleColors.TEXT_PRIMARY,
                activebackground=AppleColors.BG_PRIMARY,
                selectcolor=AppleColors.ACCENT_BLUE_LIGHT, indicatoron=0,
                padx=s(14), pady=s(6), relief=tk.FLAT, bd=1,
                highlightbackground=AppleColors.BORDER
            ).grid(row=i // 3, column=i % 3, padx=s(4), pady=s(4), sticky="we")

        for c in range(3):
            btn_frame.columnconfigure(c, weight=1)

        self._desc_label = tk.Label(frame, text="", font=AppleFonts.CAPTION,
                                    bg=AppleColors.BG_PRIMARY,
                                    fg=AppleColors.TEXT_SECONDARY, anchor="w")
        self._desc_label.pack(fill=tk.X, pady=(s(8), 0))

    # -------------------------------------------------------------- State
    def _build_state_section(self, parent):
        s = ScaleManager.s
        columns = tk.Frame(parent, bg=AppleColors.BG_PRIMARY)
        columns.pack(fill=tk.X)

        left = tk.Frame(columns, bg=AppleColors.BG_PRIMARY)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, s(8)))
        right = tk.Frame(columns, bg=AppleColors.BG_PRIMARY)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(s(8), 0))

        self._build_robot_card(left)
        self._build_board_card(left)
        self._build_screw_card(right)

    def _build_robot_card(self, parent):
        s = ScaleManager.s
        card = tk.LabelFrame(parent, text="  机器人状态  ",
                             font=AppleFonts.HEADLINE,
                             bg=AppleColors.CARD_BG, fg=AppleColors.TEXT_PRIMARY,
                             padx=s(14), pady=s(10))
        card.pack(fill=tk.X, pady=(0, s(10)))

        tk.Label(card, text="位置", font=AppleFonts.BODY_BOLD,
                 bg=AppleColors.CARD_BG, fg=AppleColors.TEXT_PRIMARY
                 ).grid(row=0, column=0, sticky="w", pady=s(2))
        self._robot_pos_var = tk.StringVar(value="initial")
        for i, (label, val) in enumerate([
            ("初始位置", "initial"), ("传送带", "conveyor"),
            ("工作台", "work_platform")
        ]):
            tk.Radiobutton(card, text=label, variable=self._robot_pos_var,
                           value=val, font=AppleFonts.BODY,
                           bg=AppleColors.CARD_BG,
                           activebackground=AppleColors.CARD_BG,
                           command=self._on_state_change
                           ).grid(row=0, column=i + 1, sticky="w", padx=s(4))

        tk.Label(card, text="持有", font=AppleFonts.BODY_BOLD,
                 bg=AppleColors.CARD_BG, fg=AppleColors.TEXT_PRIMARY
                 ).grid(row=1, column=0, sticky="w", pady=s(2))
        self._holding_var = tk.StringVar(value="none")
        for i, (label, val) in enumerate([
            ("空手", "none"), ("显示面板", "wm_board"), ("螺丝", "screw")
        ]):
            tk.Radiobutton(card, text=label, variable=self._holding_var,
                           value=val, font=AppleFonts.BODY,
                           bg=AppleColors.CARD_BG,
                           activebackground=AppleColors.CARD_BG,
                           command=self._on_state_change
                           ).grid(row=1, column=i + 1, sticky="w", padx=s(4))

    def _build_board_card(self, parent):
        s = ScaleManager.s
        card = tk.LabelFrame(parent, text="  显示面板状态  ",
                             font=AppleFonts.HEADLINE,
                             bg=AppleColors.CARD_BG, fg=AppleColors.TEXT_PRIMARY,
                             padx=s(14), pady=s(10))
        card.pack(fill=tk.X, pady=(0, s(10)))

        tk.Label(card, text="位置", font=AppleFonts.BODY_BOLD,
                 bg=AppleColors.CARD_BG, fg=AppleColors.TEXT_PRIMARY
                 ).grid(row=0, column=0, sticky="w", pady=s(2))
        self._board_pos_var = tk.StringVar(value="conveyor")
        for i, (label, val) in enumerate([
            ("传送带", "conveyor"), ("手持", "hand"),
            ("工作台", "work_platform")
        ]):
            tk.Radiobutton(card, text=label, variable=self._board_pos_var,
                           value=val, font=AppleFonts.BODY,
                           bg=AppleColors.CARD_BG,
                           activebackground=AppleColors.CARD_BG,
                           command=self._on_state_change
                           ).grid(row=0, column=i + 1, sticky="w", padx=s(4))

        self._inspected_var = tk.BooleanVar()
        self._available_var = tk.BooleanVar()
        self._placement_done_var = tk.BooleanVar()
        self._placement_ok_var = tk.BooleanVar()
        self._board_on_conveyor_var = tk.BooleanVar(value=True)
        self._board_returned_var = tk.BooleanVar()

        flags = [
            ("传送带上有面板", self._board_on_conveyor_var),
            ("已检查", self._inspected_var),
            ("面板可用", self._available_var),
            ("已放置到工作台", self._placement_done_var),
            ("放置正确", self._placement_ok_var),
            ("已归还传送带", self._board_returned_var),
        ]
        for i, (label, var) in enumerate(flags):
            r, c = divmod(i, 3)
            tk.Checkbutton(card, text=label, variable=var,
                           font=AppleFonts.BODY, bg=AppleColors.CARD_BG,
                           activebackground=AppleColors.CARD_BG,
                           command=self._on_state_change
                           ).grid(row=1 + r, column=c + 1, sticky="w",
                                  padx=s(2), pady=s(1))

    def _build_screw_card(self, parent):
        s = ScaleManager.s
        card = tk.LabelFrame(parent, text="  螺丝紧固状态 (9颗)  ",
                             font=AppleFonts.HEADLINE,
                             bg=AppleColors.CARD_BG, fg=AppleColors.TEXT_PRIMARY,
                             padx=s(14), pady=s(10))
        card.pack(fill=tk.X, pady=(0, s(10)))

        count_frame = tk.Frame(card, bg=AppleColors.CARD_BG)
        count_frame.pack(fill=tk.X, pady=(0, s(8)))
        tk.Label(count_frame, text="可用螺丝数量",
                 font=AppleFonts.BODY_BOLD,
                 bg=AppleColors.CARD_BG, fg=AppleColors.TEXT_PRIMARY
                 ).pack(side=tk.LEFT)
        self._screws_avail_var = tk.IntVar(value=9)
        tk.Spinbox(count_frame, from_=0, to=20,
                   textvariable=self._screws_avail_var, width=5,
                   font=AppleFonts.BODY,
                   command=self._on_state_change).pack(side=tk.LEFT, padx=s(8))

        tk.Label(card, text="已紧固的螺丝", font=AppleFonts.BODY_BOLD,
                 bg=AppleColors.CARD_BG, fg=AppleColors.TEXT_PRIMARY
                 ).pack(anchor=tk.W, pady=(s(4), s(4)))

        screw_frame = tk.Frame(card, bg=AppleColors.CARD_BG)
        screw_frame.pack(fill=tk.X)

        self._screw_vars = {}
        colors = {
            '1': '#E74C3C', '2': '#E67E22', '3': '#F1C40F',
            '4': '#2ECC71', '5': '#3498DB', '6': '#9B59B6',
            '7': '#1ABC9C', '8': '#E91E63', '9': '#795548'
        }

        for i in range(1, self.TOTAL_SCREWS + 1):
            label = str(i)
            var = tk.BooleanVar(value=False)
            self._screw_vars[label] = var
            f = tk.Frame(screw_frame, bg=AppleColors.CARD_BG)
            f.grid(row=(i - 1) // 3, column=(i - 1) % 3,
                   padx=s(4), pady=s(3), sticky="w")
            cv = tk.Canvas(f, width=s(12), height=s(12),
                           bg=AppleColors.CARD_BG, highlightthickness=0)
            cv.pack(side=tk.LEFT, padx=(0, s(4)))
            cv.create_oval(1, 1, s(11), s(11), fill=colors[label], outline="")
            tk.Checkbutton(f, text=f"螺丝 {label}", variable=var,
                           font=AppleFonts.BODY, bg=AppleColors.CARD_BG,
                           activebackground=AppleColors.CARD_BG,
                           command=self._on_state_change).pack(side=tk.LEFT)

        for c in range(3):
            screw_frame.columnconfigure(c, weight=1)

        qbtn_frame = tk.Frame(card, bg=AppleColors.CARD_BG)
        qbtn_frame.pack(fill=tk.X, pady=(s(8), 0))
        tk.Button(qbtn_frame, text="全选", font=AppleFonts.CAPTION,
                  command=lambda: self._set_all_screws(True), padx=s(10)
                  ).pack(side=tk.LEFT, padx=s(2))
        tk.Button(qbtn_frame, text="全不选", font=AppleFonts.CAPTION,
                  command=lambda: self._set_all_screws(False), padx=s(10)
                  ).pack(side=tk.LEFT, padx=s(2))
        tk.Button(qbtn_frame, text="仅 1-5", font=AppleFonts.CAPTION,
                  command=self._set_screws_1to5, padx=s(10)
                  ).pack(side=tk.LEFT, padx=s(2))

        self._screw_progress_label = tk.Label(
            card, text="紧固进度: 0 / 9", font=AppleFonts.CAPTION_BOLD,
            bg=AppleColors.CARD_BG, fg=AppleColors.ACCENT_BLUE)
        self._screw_progress_label.pack(anchor=tk.W, pady=(s(8), 0))

    # --------------------------------------------------------- Validation
    def _build_validation_section(self, parent):
        s = ScaleManager.s
        self._validation_frame = tk.LabelFrame(
            parent, text="  状态校验  ", font=AppleFonts.HEADLINE,
            bg=AppleColors.BG_PRIMARY, fg=AppleColors.TEXT_PRIMARY,
            padx=s(14), pady=s(10))
        self._validation_frame.pack(fill=tk.X, pady=(0, s(4)))

        self._validation_label = tk.Label(
            self._validation_frame, text="", font=AppleFonts.BODY,
            bg=AppleColors.BG_PRIMARY, fg=AppleColors.SUCCESS,
            anchor="w", justify=tk.LEFT, wraplength=700)
        self._validation_label.pack(fill=tk.X)

        self._autofix_btn = tk.Button(
            self._validation_frame, text="一键修复",
            font=AppleFonts.BODY_BOLD,
            bg=AppleColors.WARNING, fg=AppleColors.TEXT_ON_ACCENT,
            activebackground="#E08900",
            activeforeground=AppleColors.TEXT_ON_ACCENT,
            relief=tk.FLAT, padx=s(16), pady=s(4), cursor="hand2",
            command=self._auto_fix)
        self._autofix_btn.pack(anchor=tk.W, pady=(s(6), 0))
        self._autofix_btn.pack_forget()

    # -------------------------------------------------------------- Task
    def _build_task_section(self, parent):
        s = ScaleManager.s
        frame = tk.LabelFrame(parent, text="  规划任务（自动推荐）  ",
                              font=AppleFonts.HEADLINE,
                              bg=AppleColors.BG_PRIMARY,
                              fg=AppleColors.TEXT_PRIMARY,
                              padx=s(14), pady=s(10))
        frame.pack(fill=tk.X, pady=(0, s(4)))

        self._task_var = tk.StringVar(
            value='[("install_wm_display_board", "wm_board")]')
        tk.Entry(frame, textvariable=self._task_var, font=AppleFonts.BODY,
                 bg=AppleColors.INPUT_BG, fg=AppleColors.TEXT_PRIMARY,
                 relief=tk.FLAT, highlightthickness=1,
                 highlightbackground=AppleColors.BORDER
                 ).pack(fill=tk.X, pady=s(4))

        self._task_hint_label = tk.Label(
            frame, text="", font=AppleFonts.CAPTION,
            bg=AppleColors.BG_PRIMARY, fg=AppleColors.TEXT_TERTIARY)
        self._task_hint_label.pack(anchor=tk.W)

    # -------------------------------------------------------------- Preview
    def _build_preview_section(self, parent):
        s = ScaleManager.s
        frame = tk.LabelFrame(parent, text="  JSON 预览  ",
                              font=AppleFonts.HEADLINE,
                              bg=AppleColors.BG_PRIMARY,
                              fg=AppleColors.TEXT_PRIMARY,
                              padx=s(14), pady=s(10))
        frame.pack(fill=tk.X, pady=(0, s(4)))

        self._preview_text = tk.Text(
            frame, height=12,
            font=("Cascadia Code" if sys.platform == "win32"
                  else "SF Mono", 12),
            bg="#1E1E1E", fg="#D4D4D4", relief=tk.FLAT, wrap=tk.NONE,
            padx=s(10), pady=s(8), highlightthickness=0)
        self._preview_text.pack(fill=tk.BOTH, expand=True)

    # -------------------------------------------------------------- Buttons
    def _build_action_buttons(self, parent):
        s = ScaleManager.s
        btn_frame = tk.Frame(parent, bg=AppleColors.BG_PRIMARY)
        btn_frame.pack(fill=tk.X, pady=(s(16), 0))

        self._apply_btn = tk.Button(
            btn_frame, text="应用到规划器", font=AppleFonts.BODY_BOLD,
            bg=AppleColors.ACCENT_BLUE, fg=AppleColors.TEXT_ON_ACCENT,
            activebackground=AppleColors.ACCENT_BLUE_HOVER,
            activeforeground=AppleColors.TEXT_ON_ACCENT,
            relief=tk.FLAT, padx=s(24), pady=s(8), cursor="hand2",
            command=self._do_apply)
        self._apply_btn.pack(side=tk.RIGHT, padx=s(4))

        tk.Button(
            btn_frame, text="取消", font=AppleFonts.BODY_BOLD,
            bg=AppleColors.BG_TERTIARY, fg=AppleColors.TEXT_PRIMARY,
            relief=tk.FLAT, padx=s(24), pady=s(8), cursor="hand2",
            command=self.destroy).pack(side=tk.RIGHT, padx=s(4))

    # ================================================================ Logic

    def _apply_preset(self, key):
        self._is_applying_preset = True
        preset = PRESETS[key]
        st = preset["state"]
        self._preset_var.set(key)
        self._desc_label.config(text=f"  {preset['desc']}")

        self._robot_pos_var.set(st["robot_pos"])
        self._board_pos_var.set(st["board_pos"])
        self._holding_var.set(st["holding"])
        self._board_on_conveyor_var.set(st["board_on_conveyor"])
        self._available_var.set(st["board_available"])
        self._inspected_var.set(st["inspected"])
        self._placement_done_var.set(st["placement_done"])
        self._placement_ok_var.set(st["placement_ok"])
        self._screws_avail_var.set(st["screws_available"])
        self._board_returned_var.set(st["board_returned"])

        for label, var in self._screw_vars.items():
            var.set(f"hole_{label}" in st["screws_tightened"])

        self._task_var.set(preset["tasks"])
        self._is_applying_preset = False
        self._prev_snapshot = self._take_snapshot()
        self._refresh_all()

    def _take_snapshot(self):
        """Capture current state for diff detection."""
        return {
            'board_on_conveyor': self._board_on_conveyor_var.get(),
            'inspected': self._inspected_var.get(),
            'available': self._available_var.get(),
            'placement_done': self._placement_done_var.get(),
            'placement_ok': self._placement_ok_var.get(),
            'board_returned': self._board_returned_var.get(),
            'holding': self._holding_var.get(),
            'board_pos': self._board_pos_var.get(),
            'robot_pos': self._robot_pos_var.get(),
            'screws': {k: v.get() for k, v in self._screw_vars.items()},
        }

    def _on_state_change(self):
        if self._is_applying_preset or self._is_enforcing:
            return
        cur = self._take_snapshot()
        self._enforce_consistency(self._prev_snapshot, cur)
        self._prev_snapshot = self._take_snapshot()
        self._refresh_all()

    def _bind_state_var_traces(self):
        """
        对关键状态变量注册 trace 监听，确保任何写入方式
        （含 Spinbox 键盘输入）都能触发自动一致性校验。
        """
        state_vars = [
            self._robot_pos_var,
            self._holding_var,
            self._board_pos_var,
            self._board_on_conveyor_var,
            self._inspected_var,
            self._available_var,
            self._placement_done_var,
            self._placement_ok_var,
            self._board_returned_var,
            self._screws_avail_var,
            *self._screw_vars.values(),
        ]
        for var in state_vars:
            trace_id = var.trace_add("write", self._on_state_var_write)
            self._state_var_traces.append((var, trace_id))

    def _on_state_var_write(self, *_):
        self._on_state_change()

    def _refresh_all(self):
        self._auto_suggest_task()
        errors = self._validate()
        self._update_validation_display(errors)
        self._refresh_preview()

    # ---------------------------------------- Consistency enforcement
    def _enforce_consistency(self, prev=None, cur=None):
        """
        根据用户修改方向自动级联：
        - 启用高阶状态 → 向上级联，自动勾选前置依赖
        - 禁用低阶状态 → 向下级联，自动清除依赖它的高阶状态
        """
        if self._is_enforcing:
            return
        self._is_enforcing = True

        # --- 面板位置与持有始终同步（使用 elif 避免条件互相覆盖）---
        board_pos = self._board_pos_var.get()
        holding = self._holding_var.get()
        if board_pos == "hand" and holding != "wm_board":
            self._holding_var.set("wm_board")
        elif holding == "wm_board" and board_pos != "hand":
            self._board_pos_var.set("hand")

        # 检测用户变更方向
        something_disabled = False
        something_enabled = False
        if prev and cur:
            bool_keys = ['board_on_conveyor', 'inspected', 'available',
                         'placement_done', 'placement_ok', 'board_returned']
            for k in bool_keys:
                if prev.get(k) and not cur.get(k):
                    something_disabled = True
                if not prev.get(k) and cur.get(k):
                    something_enabled = True
            if prev.get('screws') and cur.get('screws'):
                for s_label in self._screw_vars:
                    if prev['screws'].get(s_label) and not cur['screws'].get(s_label):
                        something_disabled = True
                    if not prev['screws'].get(s_label) and cur['screws'].get(s_label):
                        something_enabled = True
        else:
            something_enabled = True

        if something_disabled and not something_enabled:
            self._cascade_down()
        elif something_enabled and not something_disabled:
            self._cascade_up()
        else:
            self._cascade_down()
            self._cascade_up()

        self._is_enforcing = False

    def _cascade_down(self):
        """取消低阶前提时，清除所有依赖它的高阶状态。"""
        if not self._board_on_conveyor_var.get():
            self._inspected_var.set(False)
        if not self._inspected_var.get():
            self._available_var.set(False)
        if not self._available_var.get():
            self._placement_done_var.set(False)
        if not self._placement_done_var.get():
            self._placement_ok_var.set(False)
        if not self._placement_ok_var.get():
            for v in self._screw_vars.values():
                v.set(False)
            self._board_returned_var.set(False)

    def _cascade_up(self):
        """高阶状态蕴含低阶前提，自动补全所有前置条件。"""
        if self._board_returned_var.get():
            for v in self._screw_vars.values():
                v.set(True)

        any_screw_done = any(v.get() for v in self._screw_vars.values())
        if any_screw_done:
            self._placement_ok_var.set(True)

        if self._placement_ok_var.get():
            self._placement_done_var.set(True)

        if self._placement_done_var.get():
            if self._board_pos_var.get() != "work_platform":
                self._board_pos_var.set("work_platform")
            if self._holding_var.get() == "wm_board":
                self._holding_var.set("none")
            self._inspected_var.set(True)
            self._available_var.set(True)
            self._board_on_conveyor_var.set(True)

        if self._board_pos_var.get() == "hand":
            self._inspected_var.set(True)
            self._available_var.set(True)
            self._board_on_conveyor_var.set(True)

        if self._available_var.get():
            self._inspected_var.set(True)
            self._board_on_conveyor_var.set(True)

        if self._inspected_var.get():
            self._board_on_conveyor_var.set(True)

    # ---------------------------------------- Validation
    def _validate(self):
        """
        返回 (errors, warnings)，每条带 [分组标签] 方便定位。
        errors   → 阻止应用 + 显示修复按钮
        warnings → 仅提示，不阻止应用
        """
        errors   = []
        warnings = []

        board_pos = self._board_pos_var.get()
        holding   = self._holding_var.get()

        # ── 组 A：holding / board_pos 互斥约束 ─────────────────────────────
        if holding == "wm_board" and board_pos != "hand":
            errors.append(
                "[持有冲突] 持有[显示面板]时，面板位置必须是[手持]"
                f"（当前位置：{board_pos}）")
        if board_pos == "hand" and holding != "wm_board":
            errors.append(
                "[持有冲突] 面板位置为[手持]时，持有必须是[显示面板]"
                f"（当前持有：{holding}）")

        # ── 组 B：状态前置依赖链 ────────────────────────────────────────────
        if self._available_var.get() and not self._inspected_var.get():
            errors.append("[前置缺失] [面板可用] 依赖 [已检查]，请先勾选[已检查]")
        if self._inspected_var.get() and not self._board_on_conveyor_var.get():
            errors.append("[前置缺失] [已检查] 依赖 [传送带上有面板]，请先勾选")
        if self._placement_ok_var.get() and not self._placement_done_var.get():
            errors.append("[前置缺失] [放置正确] 依赖 [已放置到工作台]，请先勾选")

        # ── 组 C：面板位置与标志位一致性 ────────────────────────────────────
        if self._placement_done_var.get() and board_pos != "work_platform":
            errors.append(
                "[位置冲突] 已勾选[已放置到工作台]，但面板位置是"
                f"[{board_pos}]，应为[work_platform]")

        # ── 组 D：螺丝状态约束 ──────────────────────────────────────────────
        any_screw = any(v.get() for v in self._screw_vars.values())
        if any_screw and not self._placement_ok_var.get():
            errors.append("[螺丝冲突] 有螺丝已紧固，但[放置正确]未勾选")
        if holding == "screw" and not self._placement_ok_var.get():
            errors.append("[螺丝冲突] 持有螺丝，但面板[放置正确]未勾选")

        all_screws = all(v.get() for v in self._screw_vars.values())
        if self._board_returned_var.get() and not all_screws:
            done = sum(1 for v in self._screw_vars.values() if v.get())
            errors.append(
                f"[归还冲突] 已勾选[已归还传送带]，但螺丝只紧固了 {done}/9 颗")

        # ── 组 E：资源警告 ───────────────────────────────────────────────────
        done_count      = sum(1 for v in self._screw_vars.values() if v.get())
        remaining_screws = self.TOTAL_SCREWS - done_count
        avail           = self._screws_avail_var.get()
        if avail < remaining_screws:
            warnings.append(
                f"[资源不足] 可用螺丝 {avail} 颗，待紧固 {remaining_screws} 颗，"
                f"差 {remaining_screws - avail} 颗，规划将失败")

        return errors, warnings

    def _check_planning_feasibility(self):
        """
        对当前状态+任务做一次可规划性试跑。
        返回 (ok: bool, msg: str)。
        仅在 _do_apply 前调用，不影响一键修复流程。
        """
        operators = htn.get_operators()
        methods   = htn.get_methods()
        if not operators or not methods:
            return True, ""

        try:
            tasks = ast.literal_eval(self._task_var.get().strip())
            if not isinstance(tasks, list):
                return False, "任务格式错误：必须是列表，例如 [(\"install_wm_display_board\", \"wm_board\")]"
        except Exception:
            return False, "任务格式错误：请使用 Python 列表/元组格式"

        state_dict = self._build_state_dict()
        s = State(state_dict.get("name", "wm_display_board_installation"))
        s.robot_pos        = state_dict["robot_pos"]
        s.obj_pos          = dict(state_dict["obj_pos"])
        s.holding          = state_dict["holding"]
        s.board_on_conveyor = state_dict["board_on_conveyor"]
        s.board_available  = state_dict["board_available"]
        s.inspected        = state_dict["inspected"]
        s.placement_done   = state_dict["placement_done"]
        s.placement_ok     = state_dict["placement_ok"]
        s.screws_available = state_dict["screws_available"]
        s.screws_tightened = list(state_dict["screws_tightened"])
        s.screw_aligned    = state_dict.get("screw_aligned")
        s.screw_inserted   = state_dict.get("screw_inserted")
        s.current_screw    = state_dict.get("current_screw")
        s.all_screws_done  = state_dict["all_screws_done"]
        s.board_returned   = state_dict["board_returned"]

        result = htn.plan(s, tasks, operators, methods, verbose=0)
        if result is False:
            return False, "当前状态与任务组合不可规划（No valid plan）。请用[自动推荐任务]重新选取任务。"
        return True, ""

    def _update_validation_display(self, result):
        if not hasattr(self, '_validation_label'):
            return
        errors, warnings = result
        s = ScaleManager.s

        if not errors and not warnings:
            self._validation_label.config(
                text="✓ 状态一致，可以规划",
                fg=AppleColors.SUCCESS)
            self._autofix_btn.pack_forget()
            self._apply_btn.config(state=tk.NORMAL)
            return

        lines = []
        if errors:
            lines.append(f"✗ 发现 {len(errors)} 处错误，点击[一键修复]自动纠正：")
            for i, e in enumerate(errors, 1):
                lines.append(f"  {i}. {e}")
        if warnings:
            if lines:
                lines.append("")
            lines.append(f"⚠ {len(warnings)} 条注意事项：")
            for i, w in enumerate(warnings, 1):
                lines.append(f"  {i}. {w}")

        self._validation_label.config(
            text="\n".join(lines),
            fg=AppleColors.ERROR if errors else AppleColors.WARNING)

        if errors:
            self._autofix_btn.pack(anchor=tk.W, pady=(s(6), 0))
            self._apply_btn.config(state=tk.DISABLED)
        else:
            self._autofix_btn.pack_forget()
            self._apply_btn.config(state=tk.NORMAL)

    def _format_validation_reasons(self, errors, warnings):
        lines = []
        if errors:
            lines.append("不一致原因：")
            for idx, msg in enumerate(errors, 1):
                lines.append(f"{idx}. {msg}")
        if warnings:
            if lines:
                lines.append("")
            lines.append("注意事项：")
            for idx, msg in enumerate(warnings, 1):
                lines.append(f"{idx}. {msg}")
        return "\n".join(lines)

    def _auto_fix(self):
        """
        一键修复 — 按优先级依次修正所有已知不一致：
          1. holding / board_pos 强制同步
          2. 持有螺丝但面板未放好 → 丢弃螺丝
          3. cascade_down（清理无效高阶状态）
          4. cascade_up（补齐前置依赖）
          5. screws_available 不足 → 自动补足
          6. 推荐任务 + 刷新校验
        所有步骤在锁定状态下执行，避免 trace 回调干扰。
        """
        # 视觉反馈：修复中
        if hasattr(self, '_autofix_btn'):
            self._autofix_btn.config(text="修复中…", state=tk.DISABLED)
            self.update_idletasks()

        self._is_enforcing = True
        self._is_applying_preset = True
        try:
            # ── 步骤 1：holding / board_pos 强制同步 ───────────────────────
            board_pos = self._board_pos_var.get()
            holding   = self._holding_var.get()

            if holding == "wm_board" and board_pos != "hand":
                if (board_pos == "work_platform"
                        and self._placement_done_var.get()):
                    self._holding_var.set("none")
                else:
                    self._board_pos_var.set("hand")
            elif board_pos == "hand" and holding != "wm_board":
                self._holding_var.set("wm_board")

            # ── 步骤 2：持有螺丝但面板未正确放置 → 丢弃螺丝 ───────────────
            if (self._holding_var.get() == "screw"
                    and not self._placement_ok_var.get()):
                self._holding_var.set("none")

            # ── 步骤 3 & 4：级联清理 + 级联补齐 ────────────────────────────
            self._cascade_down()
            self._cascade_up()

            # ── 步骤 5：可用螺丝不足 → 补足 ────────────────────────────────
            done_count = sum(1 for v in self._screw_vars.values() if v.get())
            remaining  = self.TOTAL_SCREWS - done_count
            if self._screws_avail_var.get() < remaining:
                self._screws_avail_var.set(remaining)

        except Exception as exc:
            self._is_applying_preset = False
            self._is_enforcing = False
            if hasattr(self, '_autofix_btn'):
                self._autofix_btn.config(text="一键修复", state=tk.NORMAL)
            messagebox.showerror(
                "一键修复出错",
                f"修复过程中发生异常，请截图反馈：\n\n{type(exc).__name__}: {exc}"
            )
            return

        finally:
            self._is_applying_preset = False
            self._is_enforcing = False

        # ── 步骤 6：刷新快照、推荐任务、重新校验 ───────────────────────────
        self._prev_snapshot = self._take_snapshot()
        self._refresh_all()

        # 恢复按钮文字
        if hasattr(self, '_autofix_btn'):
            self._autofix_btn.config(text="一键修复", state=tk.NORMAL)

    # ---------------------------------------- Auto task suggestion
    def _auto_suggest_task(self):
        if self._is_applying_preset:
            return

        board_pos = self._board_pos_var.get()
        holding = self._holding_var.get()
        placement_ok = self._placement_ok_var.get()
        all_screws = all(v.get() for v in self._screw_vars.values())
        board_returned = self._board_returned_var.get()

        if board_returned:
            task = '[]'
            hint = "任务已全部完成（面板已归还传送带）"
        elif all_screws and holding == "wm_board":
            task = '[("return_to_conveyor", "wm_board")]'
            hint = "归还途中 → 推荐：继续归还面板到传送带（断点续做）"
        elif all_screws:
            task = '[("return_to_conveyor", "wm_board")]'
            hint = "9颗螺丝已全部紧固 → 推荐：归还面板到传送带"
        elif holding == "screw" and placement_ok and board_pos == "work_platform":
            task = '[("install_wm_display_board", "wm_board")]'
            hint = "螺丝操作中途 → 推荐：完成当前螺丝并继续（断点续做）"
        elif placement_ok and board_pos == "work_platform":
            task = '[("fasten_all_screws",), ("return_to_conveyor", "wm_board")]'
            hint = "面板已正确放置 → 推荐：紧固螺丝 + 归还"
        elif holding == "wm_board":
            task = ('[("transport_and_place", "wm_board", "work_platform"), '
                    '("fasten_all_screws",), ("return_to_conveyor", "wm_board")]')
            hint = "已持有面板 → 推荐：搬运放置 + 紧固螺丝 + 归还"
        else:
            task = '[("install_wm_display_board", "wm_board")]'
            hint = "从头开始 → 推荐：完整安装流程"

        self._task_var.set(task)
        if self._task_hint_label is not None:
            self._task_hint_label.config(text=hint)

    # ---------------------------------------- Screw helpers
    def _set_all_screws(self, val):
        for var in self._screw_vars.values():
            var.set(val)
        self._on_state_change()

    def _set_screws_1to5(self):
        for label, var in self._screw_vars.items():
            var.set(int(label) <= 5)
        self._on_state_change()

    # ---------------------------------------- State dict / preview
    def _build_state_dict(self):
        holding_raw = self._holding_var.get()
        holding = False if holding_raw == "none" else holding_raw
        tightened = [f"hole_{lbl}" for lbl, var in self._screw_vars.items()
                     if var.get()]
        return {
            "name": "wm_display_board_installation",
            "robot_pos": self._robot_pos_var.get(),
            "obj_pos": {"wm_board": self._board_pos_var.get()},
            "holding": holding,
            "board_on_conveyor": self._board_on_conveyor_var.get(),
            "board_available": self._available_var.get(),
            "inspected": self._inspected_var.get(),
            "placement_done": self._placement_done_var.get(),
            "placement_ok": self._placement_ok_var.get(),
            "screws_available": self._screws_avail_var.get(),
            "screws_tightened": tightened,
            "screw_aligned": None,
            "screw_inserted": None,
            "current_screw": None,
            "all_screws_done": len(tightened) == self.TOTAL_SCREWS,
            "board_returned": self._board_returned_var.get(),
        }

    def _refresh_preview(self):
        if not hasattr(self, '_preview_text'):
            return
        state_dict = self._build_state_dict()
        text = json.dumps(state_dict, indent=4, ensure_ascii=False)
        self._preview_text.config(state=tk.NORMAL)
        self._preview_text.delete("1.0", tk.END)
        self._preview_text.insert(tk.END, text)

        done = sum(1 for v in self._screw_vars.values() if v.get())
        self._screw_progress_label.config(
            text=f"紧固进度: {done} / {self.TOTAL_SCREWS}")

    def _do_apply(self):
        # 兜底一致性校验
        self._enforce_consistency(self._prev_snapshot, self._take_snapshot())
        self._prev_snapshot = self._take_snapshot()
        errors, warnings = self._validate()
        if errors:
            self._refresh_all()
            reason_text = self._format_validation_reasons(errors, warnings)
            messagebox.showerror("状态校验失败", reason_text)
            return

        # 可规划性终检（仅在应用时做，不参与日常校验循环）
        ok, plan_msg = self._check_planning_feasibility()
        if not ok:
            messagebox.showerror("规划失败预警", plan_msg)
            return

        state_dict = self._build_state_dict()
        state_json = json.dumps(state_dict, indent=4, ensure_ascii=False)
        tasks_str = self._task_var.get()
        if self.on_apply:
            self.on_apply(state_json, tasks_str)
        self.destroy()

    def get_state_json(self):
        return json.dumps(self._build_state_dict(), indent=4,
                          ensure_ascii=False)

    def get_tasks(self):
        return self._task_var.get()

