"""
电视机电源模块安装 - 可视化状态配置器
TV Power Module Installation - Visual State Configurator

提供可视化界面配置不同初始状态，支持从任意阶段开始规划。
内置状态一致性自动校正和约束校验，避免配出无法规划的状态。
"""

import tkinter as tk
from tkinter import ttk
import json
import sys

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
# Preset scenarios
# ============================================================================

PRESETS = {
    "full_process": {
        "label": "从头开始",
        "desc":  "完整流程：检查 → 拿取 → 移动 → 定位 → 放置 → 紧固",
        "state": {
            "robot_pos": "initial",
            "power_com_pos": "material_box",
            "holding": "none",
            "power_com_in_box": True,
            "power_com_available": False,
            "inspected": False,
            "locating_done": False,
            "placement_done": False,
            "placement_ok": False,
            "screws_available": 10,
            "screws_tightened": [],
        },
        "tasks": '[("install_power_module", "power_com", "TV_panel")]',
    },
    "after_inspection": {
        "label": "检查完毕",
        "desc":  "电源模块已检查可用，从拿取开始",
        "state": {
            "robot_pos": "material_box",
            "power_com_pos": "material_box",
            "holding": "none",
            "power_com_in_box": True,
            "power_com_available": True,
            "inspected": True,
            "locating_done": False,
            "placement_done": False,
            "placement_ok": False,
            "screws_available": 10,
            "screws_tightened": [],
        },
        "tasks": '[("install_power_module", "power_com", "TV_panel")]',
    },
    "after_pickup": {
        "label": "已拿取模块",
        "desc":  "机器人已持有电源模块，从移动/定位开始",
        "state": {
            "robot_pos": "material_box",
            "power_com_pos": "hand",
            "holding": "power_com",
            "power_com_in_box": True,
            "power_com_available": True,
            "inspected": True,
            "locating_done": False,
            "placement_done": False,
            "placement_ok": False,
            "screws_available": 10,
            "screws_tightened": [],
        },
        "tasks": '[("position_and_place", "power_com", "TV_panel"), ("fasten_all_screws",)]',
    },
    "after_placement": {
        "label": "模块已放置",
        "desc":  "电源模块已正确放置在面板上，从螺丝紧固开始",
        "state": {
            "robot_pos": "TV_panel",
            "power_com_pos": "TV_panel",
            "holding": "none",
            "power_com_in_box": True,
            "power_com_available": True,
            "inspected": True,
            "locating_done": True,
            "placement_done": True,
            "placement_ok": True,
            "screws_available": 10,
            "screws_tightened": [],
        },
        "tasks": '[("fasten_all_screws",)]',
    },
    "partial_screws": {
        "label": "部分螺丝已紧固",
        "desc":  "A、B、C 已紧固，继续紧固剩余螺丝",
        "state": {
            "robot_pos": "TV_panel",
            "power_com_pos": "TV_panel",
            "holding": "none",
            "power_com_in_box": True,
            "power_com_available": True,
            "inspected": True,
            "locating_done": True,
            "placement_done": True,
            "placement_ok": True,
            "screws_available": 7,
            "screws_tightened": ["hole_A", "hole_B", "hole_C"],
        },
        "tasks": '[("fasten_all_screws",)]',
    },
    "placement_failed": {
        "label": "放置失败需重试",
        "desc":  "电源模块放置不正确，需要重新抓取定位",
        "state": {
            "robot_pos": "TV_panel",
            "power_com_pos": "TV_panel",
            "holding": "none",
            "power_com_in_box": True,
            "power_com_available": True,
            "inspected": True,
            "locating_done": True,
            "placement_done": True,
            "placement_ok": False,
            "screws_available": 10,
            "screws_tightened": [],
        },
        "tasks": '[("retry_placement", "power_com", "TV_panel"), ("fasten_all_screws",)]',
    },
}


# ============================================================================
# State Configurator Dialog
# ============================================================================

class StateConfiguratorDialog(tk.Toplevel):
    """可视化状态配置对话框（含状态一致性校验）"""

    def __init__(self, parent, on_apply=None):
        super().__init__(parent)
        self.on_apply = on_apply
        self.title("状态配置 - 电视机电源模块安装")
        self.geometry("820x780")
        self.configure(bg=AppleColors.BG_PRIMARY)
        self.resizable(True, True)
        self.minsize(750, 700)

        self.transient(parent)
        self.grab_set()

        self._is_applying_preset = False
        self._is_enforcing = False
        self._prev_snapshot = None

        self._build_ui()
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
        tk.Label(inner, text="电视机电源模块安装 — 状态配置",
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

        # Validation panel (between state and task)
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
        self._build_power_module_card(left)
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
            ("初始位置", "initial"), ("料箱", "material_box"),
            ("电视面板", "TV_panel")
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
            ("空手", "none"), ("电源模块", "power_com"), ("螺丝", "screw")
        ]):
            tk.Radiobutton(card, text=label, variable=self._holding_var,
                           value=val, font=AppleFonts.BODY,
                           bg=AppleColors.CARD_BG,
                           activebackground=AppleColors.CARD_BG,
                           command=self._on_state_change
                           ).grid(row=1, column=i + 1, sticky="w", padx=s(4))

    def _build_power_module_card(self, parent):
        s = ScaleManager.s
        card = tk.LabelFrame(parent, text="  电源模块状态  ",
                             font=AppleFonts.HEADLINE,
                             bg=AppleColors.CARD_BG, fg=AppleColors.TEXT_PRIMARY,
                             padx=s(14), pady=s(10))
        card.pack(fill=tk.X, pady=(0, s(10)))

        tk.Label(card, text="位置", font=AppleFonts.BODY_BOLD,
                 bg=AppleColors.CARD_BG, fg=AppleColors.TEXT_PRIMARY
                 ).grid(row=0, column=0, sticky="w", pady=s(2))
        self._pcom_pos_var = tk.StringVar(value="material_box")
        for i, (label, val) in enumerate([
            ("料箱", "material_box"), ("手持", "hand"),
            ("电视面板", "TV_panel")
        ]):
            tk.Radiobutton(card, text=label, variable=self._pcom_pos_var,
                           value=val, font=AppleFonts.BODY,
                           bg=AppleColors.CARD_BG,
                           activebackground=AppleColors.CARD_BG,
                           command=self._on_state_change
                           ).grid(row=0, column=i + 1, sticky="w", padx=s(4))

        self._inspected_var = tk.BooleanVar()
        self._available_var = tk.BooleanVar()
        self._locating_done_var = tk.BooleanVar()
        self._placement_done_var = tk.BooleanVar()
        self._placement_ok_var = tk.BooleanVar()
        self._pcom_in_box_var = tk.BooleanVar(value=True)

        flags = [
            ("料箱中有模块", self._pcom_in_box_var),
            ("已检查", self._inspected_var),
            ("模块可用", self._available_var),
            ("已定位", self._locating_done_var),
            ("已放置", self._placement_done_var),
            ("放置正确", self._placement_ok_var),
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
        card = tk.LabelFrame(parent, text="  螺丝紧固状态  ",
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
        self._screws_avail_var = tk.IntVar(value=10)
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
        colors = {'A': '#E74C3C', 'B': '#E67E22', 'C': '#F1C40F',
                  'D': '#2ECC71', 'E': '#3498DB', 'F': '#9B59B6',
                  'G': '#1ABC9C'}

        for i, label in enumerate('ABCDEFG'):
            var = tk.BooleanVar(value=False)
            self._screw_vars[label] = var
            f = tk.Frame(screw_frame, bg=AppleColors.CARD_BG)
            f.grid(row=i // 4, column=i % 4, padx=s(4), pady=s(3), sticky="w")
            cv = tk.Canvas(f, width=s(12), height=s(12),
                           bg=AppleColors.CARD_BG, highlightthickness=0)
            cv.pack(side=tk.LEFT, padx=(0, s(4)))
            cv.create_oval(1, 1, s(11), s(11), fill=colors[label], outline="")
            tk.Checkbutton(f, text=f"螺丝 {label}", variable=var,
                           font=AppleFonts.BODY, bg=AppleColors.CARD_BG,
                           activebackground=AppleColors.CARD_BG,
                           command=self._on_state_change).pack(side=tk.LEFT)

        for c in range(4):
            screw_frame.columnconfigure(c, weight=1)

        qbtn_frame = tk.Frame(card, bg=AppleColors.CARD_BG)
        qbtn_frame.pack(fill=tk.X, pady=(s(8), 0))
        tk.Button(qbtn_frame, text="全选", font=AppleFonts.CAPTION,
                  command=lambda: self._set_all_screws(True), padx=s(10)
                  ).pack(side=tk.LEFT, padx=s(2))
        tk.Button(qbtn_frame, text="全不选", font=AppleFonts.CAPTION,
                  command=lambda: self._set_all_screws(False), padx=s(10)
                  ).pack(side=tk.LEFT, padx=s(2))
        tk.Button(qbtn_frame, text="仅 A-D", font=AppleFonts.CAPTION,
                  command=self._set_screws_ad, padx=s(10)
                  ).pack(side=tk.LEFT, padx=s(2))

        self._screw_progress_label = tk.Label(
            card, text="紧固进度: 0 / 7", font=AppleFonts.CAPTION_BOLD,
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

        # Auto-fix button
        self._autofix_btn = tk.Button(
            self._validation_frame, text="一键修复",
            font=AppleFonts.BODY_BOLD,
            bg=AppleColors.WARNING, fg=AppleColors.TEXT_ON_ACCENT,
            activebackground="#E08900",
            activeforeground=AppleColors.TEXT_ON_ACCENT,
            relief=tk.FLAT, padx=s(16), pady=s(4), cursor="hand2",
            command=self._auto_fix)
        # initially hidden; shown when there are errors
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
            value='[("install_power_module", "power_com", "TV_panel")]')
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
            frame, height=10,
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
        self._pcom_pos_var.set(st["power_com_pos"])
        self._holding_var.set(st["holding"])
        self._pcom_in_box_var.set(st["power_com_in_box"])
        self._available_var.set(st["power_com_available"])
        self._inspected_var.set(st["inspected"])
        self._locating_done_var.set(st["locating_done"])
        self._placement_done_var.set(st["placement_done"])
        self._placement_ok_var.set(st["placement_ok"])
        self._screws_avail_var.set(st["screws_available"])

        for label, var in self._screw_vars.items():
            var.set(f"hole_{label}" in st["screws_tightened"])

        self._task_var.set(preset["tasks"])
        self._is_applying_preset = False
        self._prev_snapshot = self._take_snapshot()
        self._refresh_all()

    def _take_snapshot(self):
        """Capture current boolean/string variable state for diff detection."""
        return {
            'pcom_in_box': self._pcom_in_box_var.get(),
            'inspected': self._inspected_var.get(),
            'available': self._available_var.get(),
            'locating_done': self._locating_done_var.get(),
            'placement_done': self._placement_done_var.get(),
            'placement_ok': self._placement_ok_var.get(),
            'holding': self._holding_var.get(),
            'pcom_pos': self._pcom_pos_var.get(),
            'robot_pos': self._robot_pos_var.get(),
            'screws': {k: v.get() for k, v in self._screw_vars.items()},
        }

    def _on_state_change(self):
        """User changed a state variable — enforce consistency then refresh."""
        if self._is_applying_preset or self._is_enforcing:
            return
        cur = self._take_snapshot()
        self._enforce_consistency(self._prev_snapshot, cur)
        self._prev_snapshot = self._take_snapshot()
        self._refresh_all()

    def _refresh_all(self):
        errors = self._validate()
        self._update_validation_display(errors)
        self._auto_suggest_task()
        self._refresh_preview()

    # ---------------------------------------- Consistency enforcement
    def _enforce_consistency(self, prev=None, cur=None):
        """
        根据用户修改方向自动级联：
        - 用户 启用(OFF→ON) 高阶状态 → 向上级联，自动勾选前置依赖
        - 用户 禁用(ON→OFF) 低阶状态 → 向下级联，自动清除依赖它的高阶状态
        """
        if self._is_enforcing:
            return
        self._is_enforcing = True

        pcom_pos = self._pcom_pos_var.get()
        holding = self._holding_var.get()

        # --- 电源模块位置 与 持有 始终同步 ---
        if pcom_pos == "hand" and holding != "power_com":
            self._holding_var.set("power_com")
        if holding == "power_com" and pcom_pos != "hand":
            self._pcom_pos_var.set("hand")
        if pcom_pos != "hand" and holding == "power_com":
            self._holding_var.set("none")

        # 检测用户变更方向
        something_disabled = False
        something_enabled = False
        if prev and cur:
            bool_keys = ['pcom_in_box', 'inspected', 'available',
                         'locating_done', 'placement_done', 'placement_ok']
            for k in bool_keys:
                if prev.get(k) and not cur.get(k):
                    something_disabled = True
                if not prev.get(k) and cur.get(k):
                    something_enabled = True
            if prev.get('screws') and cur.get('screws'):
                for s in 'ABCDEFG':
                    if prev['screws'].get(s) and not cur['screws'].get(s):
                        something_disabled = True
                    if not prev['screws'].get(s) and cur['screws'].get(s):
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
        if not self._pcom_in_box_var.get():
            self._inspected_var.set(False)
        if not self._inspected_var.get():
            self._available_var.set(False)
        if not self._available_var.get():
            self._locating_done_var.set(False)
        if not self._locating_done_var.get():
            self._placement_done_var.set(False)
        if not self._placement_done_var.get():
            self._placement_ok_var.set(False)
        if not self._placement_ok_var.get():
            for v in self._screw_vars.values():
                v.set(False)

    def _cascade_up(self):
        """高阶状态蕴含低阶前提，自动补全所有前置条件。"""
        any_screw_done = any(v.get() for v in self._screw_vars.values())
        if any_screw_done:
            self._placement_ok_var.set(True)

        if self._placement_ok_var.get():
            self._placement_done_var.set(True)
            self._locating_done_var.set(True)

        if self._placement_done_var.get():
            if self._pcom_pos_var.get() != "TV_panel":
                self._pcom_pos_var.set("TV_panel")
            if self._holding_var.get() == "power_com":
                self._holding_var.set("none")
            self._inspected_var.set(True)
            self._available_var.set(True)
            self._pcom_in_box_var.set(True)

        if self._locating_done_var.get():
            self._inspected_var.set(True)
            self._available_var.set(True)
            self._pcom_in_box_var.set(True)

        if self._pcom_pos_var.get() == "hand":
            self._inspected_var.set(True)
            self._available_var.set(True)
            self._pcom_in_box_var.set(True)

        if self._available_var.get():
            self._inspected_var.set(True)
            self._pcom_in_box_var.set(True)

        if self._inspected_var.get():
            self._pcom_in_box_var.set(True)

        self._is_enforcing = False

    # ---------------------------------------- Validation
    def _validate(self):
        """
        检查当前状态是否存在会导致规划失败的矛盾。
        返回 (errors, warnings) 两个列表。
        """
        errors = []
        warnings = []

        pcom_pos = self._pcom_pos_var.get()
        holding = self._holding_var.get()
        robot_pos = self._robot_pos_var.get()

        # 持有与模块位置一致性
        if holding == "power_com" and pcom_pos != "hand":
            errors.append("持有电源模块时, 模块位置应为[手持]")
        if pcom_pos == "hand" and holding != "power_com":
            errors.append("模块位置为[手持]时, 持有应为[电源模块]")

        # 检查前置依赖
        if self._available_var.get() and not self._inspected_var.get():
            errors.append("[模块可用] 需要先勾选 [已检查]")
        if self._inspected_var.get() and not self._pcom_in_box_var.get():
            errors.append("[已检查] 需要勾选 [料箱中有模块]")

        if self._placement_done_var.get() and pcom_pos != "TV_panel":
            errors.append("[已放置] 但模块不在电视面板上")
        if self._placement_ok_var.get() and not self._placement_done_var.get():
            errors.append("[放置正确] 需要先勾选 [已放置]")
        if self._placement_done_var.get() and not self._locating_done_var.get():
            warnings.append("[已放置] 但未勾选 [已定位] (放置失败重试场景除外)")

        any_screw = any(v.get() for v in self._screw_vars.values())
        if any_screw and not self._placement_ok_var.get():
            errors.append("有螺丝已紧固但 [放置正确] 未勾选")

        if holding == "screw" and not self._placement_ok_var.get():
            errors.append("持有螺丝但电源模块未正确放置")

        # 可用螺丝不够
        remaining_screws = 7 - sum(1 for v in self._screw_vars.values()
                                   if v.get())
        if self._screws_avail_var.get() < remaining_screws:
            warnings.append(
                f"可用螺丝({self._screws_avail_var.get()}) "
                f"少于待紧固数量({remaining_screws}), 规划可能失败")

        # 机器人位置提醒
        if pcom_pos == "material_box" and holding == "none":
            if robot_pos != "material_box" and not self._inspected_var.get():
                pass  # moveto will handle it
        if holding == "power_com" and pcom_pos == "hand":
            pass  # normal

        return errors, warnings

    def _update_validation_display(self, result):
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
            lines.append("✗ 存在以下错误：")
            for e in errors:
                lines.append(f"    • {e}")
        if warnings:
            if errors:
                lines.append("")
            lines.append("⚠ 注意：")
            for w in warnings:
                lines.append(f"    • {w}")

        self._validation_label.config(
            text="\n".join(lines),
            fg=AppleColors.ERROR if errors else AppleColors.WARNING)

        if errors:
            self._autofix_btn.pack(anchor=tk.W, pady=(s(6), 0))
            self._apply_btn.config(state=tk.DISABLED)
        else:
            self._autofix_btn.pack_forget()
            self._apply_btn.config(state=tk.NORMAL)

    def _auto_fix(self):
        """重新强制执行一致性校正，修复所有已知错误。"""
        self._is_enforcing = False
        self._cascade_up()
        self._prev_snapshot = self._take_snapshot()
        self._refresh_all()

    # ---------------------------------------- Auto task suggestion
    def _auto_suggest_task(self):
        """根据当前状态自动推荐合适的规划任务。"""
        if self._is_applying_preset:
            return

        pcom_pos = self._pcom_pos_var.get()
        holding = self._holding_var.get()
        placement_ok = self._placement_ok_var.get()
        placement_done = self._placement_done_var.get()
        any_screw = any(v.get() for v in self._screw_vars.values())
        all_screws = all(v.get() for v in self._screw_vars.values())

        if all_screws:
            task = '[]'
            hint = "所有螺丝已紧固，安装已完成"
        elif placement_ok and pcom_pos == "TV_panel":
            task = '[("fasten_all_screws",)]'
            hint = "电源模块已正确放置 → 推荐：紧固螺丝"
        elif (placement_done and not placement_ok
              and pcom_pos == "TV_panel"):
            task = ('[("retry_placement", "power_com", "TV_panel"), '
                    '("fasten_all_screws",)]')
            hint = "放置不正确 → 推荐：重试放置 + 紧固螺丝"
        elif holding == "power_com":
            task = ('[("position_and_place", "power_com", "TV_panel"), '
                    '("fasten_all_screws",)]')
            hint = "已持有电源模块 → 推荐：定位放置 + 紧固螺丝"
        else:
            task = '[("install_power_module", "power_com", "TV_panel")]'
            hint = "从头开始 → 推荐：完整安装流程"

        self._task_var.set(task)
        self._task_hint_label.config(text=hint)

    # ---------------------------------------- Screw helpers
    def _set_all_screws(self, val):
        for var in self._screw_vars.values():
            var.set(val)
        self._on_state_change()

    def _set_screws_ad(self):
        for label, var in self._screw_vars.items():
            var.set(label in ('A', 'B', 'C', 'D'))
        self._on_state_change()

    # ---------------------------------------- State dict / preview
    def _build_state_dict(self):
        holding_raw = self._holding_var.get()
        holding = False if holding_raw == "none" else holding_raw
        tightened = [f"hole_{lbl}" for lbl, var in self._screw_vars.items()
                     if var.get()]
        return {
            "name": "tv_power_module_installation",
            "robot_pos": self._robot_pos_var.get(),
            "obj_pos": {"power_com": self._pcom_pos_var.get()},
            "holding": holding,
            "power_com_in_box": self._pcom_in_box_var.get(),
            "power_com_available": self._available_var.get(),
            "inspected": self._inspected_var.get(),
            "locating_done": self._locating_done_var.get(),
            "placement_done": self._placement_done_var.get(),
            "placement_ok": self._placement_ok_var.get(),
            "screws_available": self._screws_avail_var.get(),
            "screws_tightened": tightened,
            "screw_aligned": None,
            "screw_inserted": None,
            "current_screw": None,
            "all_screws_done": len(tightened) == 7,
        }

    def _refresh_preview(self):
        state_dict = self._build_state_dict()
        text = json.dumps(state_dict, indent=4, ensure_ascii=False)
        self._preview_text.config(state=tk.NORMAL)
        self._preview_text.delete("1.0", tk.END)
        self._preview_text.insert(tk.END, text)

        done = sum(1 for v in self._screw_vars.values() if v.get())
        self._screw_progress_label.config(text=f"紧固进度: {done} / 7")

    def _do_apply(self):
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
