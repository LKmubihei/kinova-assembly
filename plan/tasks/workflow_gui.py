"""
power_panel 装配任务的中文 HTN 可视化控制台。
"""

from __future__ import annotations

import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any, Dict, List, Optional

_SRC_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(_SRC_ROOT))

from plan.tasks.operators import EXPECTED_HOLES
from plan.tasks.workflow import (
    WorkflowPerception,
    build_state,
    build_state_from_perception,
    collect_live_perception,
    default_hole_positions,
    execute_live_hole_visit,
    format_action,
    format_plan,
    format_state_summary,
    load_perception_bundle,
    parse_panel_sequence,
    plan_installation,
    run_perception_from_files,
    simulate_plan,
)

FLOW_COLORS = {
    "pending": "#D7DBE0",
    "success": "#2E8B57",
    "warning": "#F39C12",
    "manual": "#C0392B",
    "active": "#1976D2",
}


class HTNWorkflowGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("HTN 装配流程控制台")
        self.root.geometry("1680x1040")
        self.root.configure(bg="#F5F6F8")

        self.style = ttk.Style()
        try:
            self.style.theme_use("clam")
        except tk.TclError:
            pass

        self.bin_image_path: Optional[Path] = None
        self.home_image_path: Optional[Path] = None
        self.perception: Optional[WorkflowPerception] = None
        self.initial_state = None
        self.plan_result: Optional[Any] = None
        self.final_state = None
        self.trace: List[Dict[str, Any]] = []
        self.plan_generation_count = 0

        self.status_var = tk.StringVar(
            value="默认面向真实机器人。请先真实采集，或上传图像/识别结果后再规划。"
        )
        self.plan_info_var = tk.StringVar(value="尚未生成计划。")
        self.save_dir_var = tk.StringVar(value="结果目录：未生成")

        self.bin_result_var = tk.BooleanVar(value=True)
        self.panel_result_var = tk.BooleanVar(value=True)
        self.panel_sequence_var = tk.StringVar(value="成功")
        self.max_attempts_var = tk.IntVar(value=2)
        self.hole_vars = {
            hole: tk.BooleanVar(value=True)
            for hole in EXPECTED_HOLES
        }

        self.stage_widgets: Dict[str, Dict[str, Any]] = {}

        self._build_ui()
        self._draw_flow()
        self._refresh_override_hint()

    def _build_ui(self) -> None:
        self._build_toolbar()
        self._build_flow_header()
        self._build_body()

    def _build_toolbar(self) -> None:
        toolbar = tk.Frame(self.root, bg="#F5F6F8", padx=12, pady=10)
        toolbar.pack(fill=tk.X)

        buttons = [
            ("真实采集并识别", self._run_live_capture),
            ("导入识别结果", self._import_perception_result),
            ("上传料箱图", self._select_bin_image),
            ("上传背板图", self._select_home_image),
            ("从上传图像识别", self._run_file_perception),
            ("生成规划", self._build_plan),
            ("重规划", self._replan_plan),
            ("真实执行孔位巡检", self._execute_live_visit),
            ("清空结果", self._reset_all),
        ]

        for text, command in buttons:
            tk.Button(
                toolbar,
                text=text,
                command=command,
                bg="#FFFFFF",
                fg="#1F2937",
                activebackground="#E8EEF7",
                relief=tk.GROOVE,
                padx=10,
                pady=6,
            ).pack(side=tk.LEFT, padx=4)

        tk.Label(
            toolbar,
            textvariable=self.save_dir_var,
            bg="#F5F6F8",
            fg="#4B5563",
            anchor="w",
        ).pack(side=tk.RIGHT, padx=4)

    def _build_flow_header(self) -> None:
        header = tk.Frame(self.root, bg="#F5F6F8", padx=12, pady=6)
        header.pack(fill=tk.X)

        self.flow_canvas = tk.Canvas(
            header,
            height=120,
            bg="#FFFFFF",
            highlightthickness=1,
            highlightbackground="#D6DBE3",
        )
        self.flow_canvas.pack(fill=tk.X)

        tk.Label(
            header,
            textvariable=self.status_var,
            bg="#F5F6F8",
            fg="#1F2937",
            anchor="w",
            justify=tk.LEFT,
        ).pack(fill=tk.X, pady=(8, 0))

    def _build_body(self) -> None:
        body = ttk.Notebook(self.root)
        body.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))

        self.result_tab = tk.Frame(body, bg="#F7F8FA")
        self.override_tab = tk.Frame(body, bg="#F7F8FA")
        self.plan_tab = tk.Frame(body, bg="#F7F8FA")
        self.trace_tab = tk.Frame(body, bg="#F7F8FA")

        body.add(self.result_tab, text="识别结果摘要")
        body.add(self.override_tab, text="重规划修正")
        body.add(self.plan_tab, text="规划结果")
        body.add(self.trace_tab, text="执行记录")

        self._build_result_tab()
        self._build_override_tab()
        self._build_plan_tab()
        self._build_trace_tab()

    def _build_result_tab(self) -> None:
        container = tk.Frame(self.result_tab, bg="#F7F8FA", padx=10, pady=10)
        container.pack(fill=tk.BOTH, expand=True)
        stages_container = tk.Frame(container, bg="#F7F8FA")
        stages_container.pack(fill=tk.BOTH, expand=True)

        stages = [
            ("bin", "Step1 料箱检测"),
            ("panel", "Step2 放置验证"),
            ("screw", "Step3 螺丝孔检测"),
        ]

        for column, (key, title) in enumerate(stages):
            frame = tk.LabelFrame(
                stages_container,
                text=title,
                bg="#FFFFFF",
                fg="#111827",
                padx=10,
                pady=10,
            )
            frame.grid(row=0, column=column, sticky="nsew", padx=6, pady=6)
            stages_container.grid_columnconfigure(column, weight=1)

            summary_var = tk.StringVar(value="尚未执行识别。")
            tk.Label(
                frame,
                textvariable=summary_var,
                bg="#FFFFFF",
                fg="#374151",
                justify=tk.LEFT,
                anchor="w",
                wraplength=460,
            ).pack(fill=tk.X, pady=(0, 8))

            image_row = tk.Frame(frame, bg="#FFFFFF")
            image_row.pack(fill=tk.X, expand=False)

            detail_var = tk.StringVar(value="尚未生成结果文件。")
            tk.Label(
                image_row,
                textvariable=detail_var,
                bg="#F9FAFB",
                fg="#6B7280",
                justify=tk.LEFT,
                anchor="w",
                wraplength=480,
                relief=tk.GROOVE,
                padx=10,
                pady=10,
            ).pack(fill=tk.X, expand=True)

            self.stage_widgets[key] = {
                "summary_var": summary_var,
                "detail_var": detail_var,
                "title": title,
            }

    def _build_override_tab(self) -> None:
        container = tk.Frame(self.override_tab, bg="#F7F8FA", padx=14, pady=14)
        container.pack(fill=tk.BOTH, expand=True)
        container.grid_columnconfigure(0, weight=1)
        container.grid_columnconfigure(1, weight=1)

        left = tk.LabelFrame(container, text="人工修正", bg="#FFFFFF", fg="#111827", padx=12, pady=12)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        right = tk.LabelFrame(container, text="修正说明", bg="#FFFFFF", fg="#111827", padx=12, pady=12)
        right.grid(row=0, column=1, sticky="nsew")

        tk.Label(left, text="料箱是否有 power_panel", bg="#FFFFFF", fg="#111827").grid(row=0, column=0, sticky="w")
        self._add_bool_selector(left, 1, self.bin_result_var)

        tk.Label(left, text="放置验证是否通过", bg="#FFFFFF", fg="#111827").grid(row=2, column=0, sticky="w", pady=(12, 0))
        self._add_bool_selector(left, 3, self.panel_result_var)

        tk.Label(left, text="放置验证序列", bg="#FFFFFF", fg="#111827").grid(row=4, column=0, sticky="w", pady=(12, 0))
        tk.Entry(left, textvariable=self.panel_sequence_var, width=32).grid(row=5, column=0, sticky="w")

        tk.Label(
            left,
            text="示例：成功 或 失败,成功。用于描述重试后的验证结果。",
            bg="#FFFFFF",
            fg="#6B7280",
            justify=tk.LEFT,
        ).grid(row=6, column=0, sticky="w", pady=(4, 0))

        tk.Label(left, text="最大放置尝试次数", bg="#FFFFFF", fg="#111827").grid(row=7, column=0, sticky="w", pady=(12, 0))
        tk.Spinbox(left, from_=1, to=5, textvariable=self.max_attempts_var, width=8).grid(row=8, column=0, sticky="w")

        tk.Label(left, text="可用孔位", bg="#FFFFFF", fg="#111827").grid(row=9, column=0, sticky="w", pady=(14, 0))
        holes_frame = tk.Frame(left, bg="#FFFFFF")
        holes_frame.grid(row=10, column=0, sticky="w")

        for index, hole in enumerate(EXPECTED_HOLES):
            tk.Checkbutton(
                holes_frame,
                text=hole,
                variable=self.hole_vars[hole],
                bg="#FFFFFF",
                activebackground="#FFFFFF",
                command=self._refresh_override_hint,
            ).grid(row=index // 4, column=index % 4, sticky="w", padx=8, pady=4)

        tk.Button(
            left,
            text="根据当前修正生成计划",
            command=self._build_plan,
            bg="#1D4ED8",
            fg="#FFFFFF",
            activebackground="#1E40AF",
            padx=12,
            pady=8,
        ).grid(row=11, column=0, sticky="w", pady=(16, 0))

        self.override_hint_var = tk.StringVar(value="手工补孔位将使用模板坐标，仅用于规划和模拟。")
        tk.Label(
            right,
            textvariable=self.override_hint_var,
            bg="#FFFFFF",
            fg="#374151",
            justify=tk.LEFT,
            anchor="nw",
            wraplength=520,
        ).pack(fill=tk.X)

        tips = [
            "1. 用户看到的核心流程应是：采集或导入识别结果 -> 规划 -> 修正 -> 重规划。",
            "2. 如果放置验证序列填写“失败,成功”，HTN 会先走失败分支，再自动重规划到重试路径。",
            "3. 如果孔位勾选不全，HTN 会切到人工复核分支，而不是继续盲目拧螺丝。",
            "4. 手工补孔位只用于规划和重规划，不直接替代真实视觉定位。",
        ]
        for tip in tips:
            tk.Label(
                right,
                text=tip,
                bg="#FFFFFF",
                fg="#4B5563",
                justify=tk.LEFT,
                anchor="w",
                wraplength=520,
            ).pack(fill=tk.X, pady=(10, 0))

    def _build_plan_tab(self) -> None:
        container = tk.Frame(self.plan_tab, bg="#F7F8FA", padx=12, pady=12)
        container.pack(fill=tk.BOTH, expand=True)
        container.grid_columnconfigure(0, weight=1)
        container.grid_columnconfigure(1, weight=1)
        container.grid_rowconfigure(0, weight=1)

        state_frame = tk.LabelFrame(container, text="初始状态", bg="#FFFFFF", fg="#111827", padx=10, pady=10)
        state_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        plan_frame = tk.LabelFrame(container, text="计划步骤", bg="#FFFFFF", fg="#111827", padx=10, pady=10)
        plan_frame.grid(row=0, column=1, sticky="nsew")

        self.state_text = tk.Text(state_frame, wrap=tk.WORD, bg="#F9FAFB", fg="#111827", height=28)
        self.state_text.pack(fill=tk.BOTH, expand=True)

        tk.Label(plan_frame, textvariable=self.plan_info_var, bg="#FFFFFF", fg="#374151", anchor="w").pack(fill=tk.X)

        self.plan_tree = ttk.Treeview(plan_frame, columns=("step", "action"), show="headings", height=24)
        self.plan_tree.heading("step", text="步骤")
        self.plan_tree.heading("action", text="动作")
        self.plan_tree.column("step", width=70, anchor="center")
        self.plan_tree.column("action", width=650, anchor="w")
        self.plan_tree.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

    def _build_trace_tab(self) -> None:
        container = tk.Frame(self.trace_tab, bg="#F7F8FA", padx=12, pady=12)
        container.pack(fill=tk.BOTH, expand=True)
        container.grid_columnconfigure(0, weight=3)
        container.grid_columnconfigure(1, weight=2)
        container.grid_rowconfigure(0, weight=1)

        trace_frame = tk.LabelFrame(container, text="执行轨迹", bg="#FFFFFF", fg="#111827", padx=10, pady=10)
        trace_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        final_frame = tk.LabelFrame(container, text="最终状态", bg="#FFFFFF", fg="#111827", padx=10, pady=10)
        final_frame.grid(row=0, column=1, sticky="nsew")

        self.trace_tree = ttk.Treeview(
            trace_frame,
            columns=("step", "action", "status", "changes"),
            show="headings",
            height=24,
        )
        self.trace_tree.heading("step", text="步骤")
        self.trace_tree.heading("action", text="动作")
        self.trace_tree.heading("status", text="结果")
        self.trace_tree.heading("changes", text="状态变化")
        self.trace_tree.column("step", width=70, anchor="center")
        self.trace_tree.column("action", width=220, anchor="w")
        self.trace_tree.column("status", width=80, anchor="center")
        self.trace_tree.column("changes", width=620, anchor="w")
        self.trace_tree.pack(fill=tk.BOTH, expand=True)

        self.final_state_text = tk.Text(final_frame, wrap=tk.WORD, bg="#F9FAFB", fg="#111827", height=28)
        self.final_state_text.pack(fill=tk.BOTH, expand=True)

    def _add_bool_selector(self, parent: tk.Widget, row: int, variable: tk.BooleanVar) -> None:
        holder = tk.Frame(parent, bg="#FFFFFF")
        holder.grid(row=row, column=0, sticky="w")
        tk.Radiobutton(
            holder,
            text="是",
            variable=variable,
            value=True,
            bg="#FFFFFF",
            activebackground="#FFFFFF",
            command=self._refresh_override_hint,
        ).pack(side=tk.LEFT, padx=(0, 10))
        tk.Radiobutton(
            holder,
            text="否",
            variable=variable,
            value=False,
            bg="#FFFFFF",
            activebackground="#FFFFFF",
            command=self._refresh_override_hint,
        ).pack(side=tk.LEFT)

    def _select_bin_image(self) -> None:
        filename = filedialog.askopenfilename(
            title="选择料箱图像",
            filetypes=[("图像文件", "*.png *.jpg *.jpeg *.bmp")],
        )
        if filename:
            self.bin_image_path = Path(filename)
            self.status_var.set(f"已选择料箱图：{self.bin_image_path}")

    def _select_home_image(self) -> None:
        filename = filedialog.askopenfilename(
            title="选择背板图像",
            filetypes=[("图像文件", "*.png *.jpg *.jpeg *.bmp")],
        )
        if filename:
            self.home_image_path = Path(filename)
            self.status_var.set(f"已选择背板图：{self.home_image_path}")

    def _run_file_perception(self) -> None:
        if not self.bin_image_path or not self.home_image_path:
            messagebox.showwarning("缺少图像", "请先分别上传料箱图和背板图。")
            return
        perception = run_perception_from_files(self.bin_image_path, self.home_image_path)
        self._apply_perception(perception, "已完成上传图像识别。")

    def _import_perception_result(self) -> None:
        filename = filedialog.askopenfilename(
            title="导入识别结果",
            filetypes=[("识别结果", "*.json")],
        )
        if not filename:
            return
        try:
            perception = load_perception_bundle(Path(filename))
        except Exception as exc:
            messagebox.showerror("导入失败", str(exc))
            self.status_var.set(f"识别结果导入失败：{exc}")
            return
        perception.source_mode = "imported"
        self._apply_perception(perception, "已导入识别结果，可直接规划或重规划。")

    def _run_live_capture(self) -> None:
        if not messagebox.askyesno("真实采集", "将直接使用已连接机器人和相机进行采集，是否继续？"):
            return
        try:
            perception = collect_live_perception()
        except Exception as exc:
            messagebox.showerror("真实采集失败", str(exc))
            self.status_var.set(f"真实采集失败：{exc}")
            return
        self._apply_perception(perception, "已完成真实采集和识别。")

    def _apply_perception(self, perception: WorkflowPerception, status_text: str) -> None:
        self.perception = perception
        self.initial_state = None
        self.plan_result = None
        self.final_state = None
        self.trace = []
        self.plan_generation_count = 0

        self.save_dir_var.set(f"结果目录：{perception.save_dir}")
        self.status_var.set(status_text)
        self._fill_override_controls_from_perception()
        self._update_result_tab()
        self._clear_plan_widgets()
        self._clear_trace_widgets()
        self._draw_flow()

    def _fill_override_controls_from_perception(self) -> None:
        if not self.perception:
            return
        self.bin_result_var.set(self.perception.bin_result)
        self.panel_result_var.set(self.perception.panel_result)
        self.panel_sequence_var.set("成功" if self.perception.panel_result else "失败")
        self.max_attempts_var.set(2)
        detected_holes = set(self.perception.screw_holes.keys())
        for hole, variable in self.hole_vars.items():
            variable.set(hole in detected_holes)
        self._refresh_override_hint()

    def _update_result_tab(self) -> None:
        if not self.perception:
            return

        stage_map = {
            "bin": self.perception.bin_step,
            "panel": self.perception.panel_step,
            "screw": self.perception.screw_step,
        }
        for key, step in stage_map.items():
            widgets = self.stage_widgets[key]
            widgets["summary_var"].set(step.message)
            detail_lines = []
            if step.raw_path:
                detail_lines.append(f"原始结果文件: {step.raw_path}")
            if step.annotated_path:
                detail_lines.append(f"识别结果文件: {step.annotated_path}")
            if not detail_lines:
                detail_lines.append("当前阶段没有图像文件。")
            widgets["detail_var"].set("\n".join(detail_lines))

    def _refresh_override_hint(self) -> None:
        selected_holes = [hole for hole, variable in self.hole_vars.items() if variable.get()]
        if selected_holes:
            hole_text = ", ".join(selected_holes)
        else:
            hole_text = "无"
        self.override_hint_var.set(
            "当前人工孔位选择："
            + hole_text
            + "\n手工补孔位将自动使用模板坐标，仅用于 HTN 规划和模拟执行。"
            + "\n真实执行会忽略没有真实识别坐标的补孔位。"
        )
        self._draw_flow()

    def _collect_override_holes(self) -> Dict[str, Any]:
        selected_holes = {hole for hole, variable in self.hole_vars.items() if variable.get()}
        template_holes = default_hole_positions(selected_holes)

        if self.perception:
            for hole, position in self.perception.screw_holes.items():
                if hole in selected_holes:
                    template_holes[hole] = tuple(position)
        return template_holes

    def _build_plan(self, is_replan: bool = False) -> None:
        try:
            panel_sequence = parse_panel_sequence(
                self.panel_sequence_var.get(),
                self.panel_result_var.get(),
            )
        except ValueError as exc:
            messagebox.showerror("输入错误", str(exc))
            return

        screw_holes = self._collect_override_holes()
        max_attempts = max(1, int(self.max_attempts_var.get()))

        if self.perception:
            overrides = {
                "bin_result": self.bin_result_var.get(),
                "panel_result": self.panel_result_var.get(),
                "screw_holes": screw_holes,
            }
            self.initial_state = build_state_from_perception(
                self.perception,
                panel_sequence=panel_sequence,
                max_placement_attempts=max_attempts,
                overrides=overrides,
            )
        else:
            self.initial_state = build_state(
                bin_result=self.bin_result_var.get(),
                panel_result=self.panel_result_var.get(),
                screw_holes=screw_holes,
                panel_sequence=panel_sequence,
                max_placement_attempts=max_attempts,
            )

        self.plan_result = plan_installation(self.initial_state)
        self.final_state = None
        self.trace = []
        self.plan_generation_count += 1

        self._update_plan_widgets()
        self._clear_trace_widgets()
        self._draw_flow()

        if self.plan_result is False:
            prefix = "重规划" if is_replan else "规划"
            self.status_var.set(f"HTN {prefix}失败，请检查人工修正设置。")
        else:
            step_count = len(self.plan_result) if isinstance(self.plan_result, list) else 0
            stage = "重规划" if is_replan or self.plan_generation_count > 1 else "规划"
            self.status_var.set(f"HTN {stage}完成，第 {self.plan_generation_count} 轮，共 {step_count} 步。")

    def _replan_plan(self) -> None:
        if self.perception is None and self.initial_state is None:
            messagebox.showwarning("缺少识别结果", "请先真实采集、上传图像识别，或导入识别结果。")
            return
        self._build_plan(is_replan=True)

    def _simulate_plan(self) -> None:
        if self.initial_state is None:
            messagebox.showwarning("缺少初始状态", "请先生成 HTN 计划。")
            return
        if self.plan_result is False or self.plan_result is None:
            messagebox.showwarning("缺少计划", "当前没有可模拟执行的计划。")
            return

        self.final_state, self.trace = simulate_plan(self.initial_state, self.plan_result)
        self._update_trace_widgets()
        self._draw_flow()

        if getattr(self.final_state, "manual_review_required", False):
            self.status_var.set("模拟执行结束：流程进入人工复核分支。")
        else:
            self.status_var.set("模拟执行结束。")

    def _execute_live_visit(self) -> None:
        if self.perception is None:
            messagebox.showwarning("缺少识别结果", "请先完成真实采集并识别。")
            return
        if self.perception.source_mode != "live":
            messagebox.showwarning("安全限制", "真实执行只允许基于“真实采集并识别”的结果。")
            return
        if self.plan_result is None:
            messagebox.showwarning("缺少计划", "请先生成 HTN 计划，再执行真实控制。")
            return
        if self.plan_result is False:
            messagebox.showwarning("计划失败", "当前 HTN 无可行计划，不能执行真实控制。")
            return

        selected_holes = {
            hole for hole, variable in self.hole_vars.items()
            if variable.get()
        }
        screw_holes = {
            hole: tuple(position)
            for hole, position in self.perception.screw_holes.items()
            if hole in selected_holes
        }
        ignored_holes = sorted(selected_holes - set(screw_holes.keys()))

        if not screw_holes:
            messagebox.showwarning("缺少真实孔位", "当前没有可用于真实执行的孔位识别结果。")
            return

        message = "将按当前真实识别孔位驱动机械臂依次移动到各孔位上方。"
        if ignored_holes:
            message += "\n以下仅人工勾选、但无真实识别坐标的孔位会被忽略："
            message += ", ".join(ignored_holes)
        message += "\n是否继续？"
        if not messagebox.askyesno("真实执行孔位巡检", message):
            return

        def report_status(text: str) -> None:
            self.status_var.set(text)
            self.root.update_idletasks()

        try:
            self.final_state = None
            self.trace = execute_live_hole_visit(
                screw_holes,
                dwell_seconds=3.0,
                report_hook=report_status,
            )
        except Exception as exc:
            messagebox.showerror("真实执行失败", str(exc))
            self.status_var.set(f"真实执行失败：{exc}")
            return

        self._update_trace_widgets()
        self._draw_flow()

        if ignored_holes:
            self.status_var.set(
                "真实孔位巡检完成；未执行的人工补孔位："
                + ", ".join(ignored_holes)
            )
        else:
            self.status_var.set("真实孔位巡检执行完成。")

    def _update_plan_widgets(self) -> None:
        self.state_text.delete("1.0", tk.END)
        self.plan_tree.delete(*self.plan_tree.get_children())

        if self.initial_state is not None:
            self.state_text.insert(tk.END, format_state_summary(self.initial_state))

        if self.plan_result is False:
            self.plan_info_var.set(f"第 {self.plan_generation_count} 轮规划失败：未找到可行计划。")
            return

        if self.plan_result is None:
            self.plan_info_var.set("尚未生成计划。")
            return

        summary_line = format_plan(self.plan_result).splitlines()[0]
        self.plan_info_var.set(f"第 {self.plan_generation_count} 轮规划：{summary_line}")
        for index, action in enumerate(self.plan_result, start=1):
            self.plan_tree.insert("", tk.END, values=(index, format_action(action)))

    def _update_trace_widgets(self) -> None:
        self.trace_tree.delete(*self.trace_tree.get_children())
        self.final_state_text.delete("1.0", tk.END)

        for item in self.trace:
            self.trace_tree.insert(
                "",
                tk.END,
                values=(
                    item["step"],
                    item["action"],
                    item["status"],
                    "；".join(item["changes"]),
                ),
            )

        if self.final_state is not None:
            self.final_state_text.insert(tk.END, format_state_summary(self.final_state))

    def _clear_plan_widgets(self) -> None:
        self.plan_info_var.set("尚未生成计划。")
        self.state_text.delete("1.0", tk.END)
        self.plan_tree.delete(*self.plan_tree.get_children())

    def _clear_trace_widgets(self) -> None:
        self.trace_tree.delete(*self.trace_tree.get_children())
        self.final_state_text.delete("1.0", tk.END)

    def _reset_all(self) -> None:
        self.bin_image_path = None
        self.home_image_path = None
        self.perception = None
        self.initial_state = None
        self.plan_result = None
        self.final_state = None
        self.trace = []
        self.plan_generation_count = 0
        self.status_var.set("已清空当前结果。请优先使用真实采集或导入识别结果。")
        self.save_dir_var.set("结果目录：未生成")
        self.plan_info_var.set("尚未生成计划。")
        self.panel_sequence_var.set("成功")
        self.max_attempts_var.set(2)
        for variable in self.hole_vars.values():
            variable.set(True)
        for widgets in self.stage_widgets.values():
            widgets["summary_var"].set("尚未执行识别。")
            widgets["detail_var"].set("尚未生成结果文件。")
        self._clear_plan_widgets()
        self._clear_trace_widgets()
        self._refresh_override_hint()
        self._draw_flow()

    def _draw_flow(self) -> None:
        canvas = self.flow_canvas
        canvas.delete("all")
        width = canvas.winfo_width()
        if width < 200:
            width = 1500
        height = 120
        margin = 40
        names = [
            ("料箱检测", self._status_for_bin()),
            ("抓取与放置", self._status_for_placement()),
            ("孔位检测", self._status_for_holes()),
            ("HTN 规划", self._status_for_plan()),
            ("真实执行", self._status_for_execution()),
        ]

        step = (width - margin * 2) / max(len(names) - 1, 1)
        positions = []
        for index, (_, info) in enumerate(names):
            positions.append((margin + index * step, 56, info))

        for index in range(len(positions) - 1):
            x1, y1, _ = positions[index]
            x2, y2, _ = positions[index + 1]
            canvas.create_line(x1 + 44, y1, x2 - 44, y2, fill="#CBD5E1", width=3)

        for (title, info), (x, y, _) in zip(names, positions):
            color = FLOW_COLORS[info["status"]]
            canvas.create_oval(x - 44, y - 24, x + 44, y + 24, fill=color, outline="")
            canvas.create_text(x, y, text=title, fill="#FFFFFF", font=("Noto Sans CJK SC", 11, "bold"))
            canvas.create_text(
                x,
                y + 40,
                text=info["note"],
                fill="#334155",
                font=("Noto Sans CJK SC", 10),
                width=210,
            )

    def _status_for_bin(self) -> Dict[str, str]:
        if not self.perception and self.plan_result is None:
            return {"status": "pending", "note": "等待真实采集/导入"}
        if self.bin_result_var.get():
            return {"status": "success", "note": "有料，可继续"}
        return {"status": "manual", "note": "无料或识别失败"}

    def _status_for_placement(self) -> Dict[str, str]:
        try:
            sequence = parse_panel_sequence(self.panel_sequence_var.get(), self.panel_result_var.get())
        except ValueError:
            return {"status": "manual", "note": "验证序列配置错误"}

        if not self.perception and self.plan_result is None:
            return {"status": "pending", "note": "等待识别"}
        if sequence[0] and self.panel_result_var.get():
            return {"status": "success", "note": "首轮验证通过"}
        if any(sequence):
            return {"status": "warning", "note": "首轮失败，可通过重试恢复"}
        return {"status": "manual", "note": "需要人工确认放置状态"}

    def _status_for_holes(self) -> Dict[str, str]:
        selected = [hole for hole, variable in self.hole_vars.items() if variable.get()]
        if not self.perception and self.plan_result is None:
            return {"status": "pending", "note": "等待识别"}
        if len(selected) == len(EXPECTED_HOLES):
            return {"status": "success", "note": "孔位完整"}
        if selected:
            return {"status": "manual", "note": f"仅 {len(selected)}/{len(EXPECTED_HOLES)} 个孔位"}
        return {"status": "manual", "note": "无有效孔位"}

    def _status_for_plan(self) -> Dict[str, str]:
        if self.plan_result is None:
            return {"status": "pending", "note": "等待规划"}
        if self.plan_result is False:
            return {"status": "manual", "note": "无可行计划"}
        note = f"第{self.plan_generation_count}轮 {len(self.plan_result)}步"
        return {"status": "success", "note": note}

    def _status_for_execution(self) -> Dict[str, str]:
        if not self.trace:
            return {"status": "pending", "note": "等待真实执行"}
        if self.final_state is not None and getattr(self.final_state, "manual_review_required", False):
            reason = getattr(self.final_state, "manual_review_reason", "需要人工复核")
            return {"status": "manual", "note": reason}
        return {"status": "success", "note": "执行记录已生成"}


def main() -> None:
    root = tk.Tk()
    app = HTNWorkflowGUI(root)
    root.update_idletasks()
    app._draw_flow()
    root.mainloop()


if __name__ == "__main__":
    main()
