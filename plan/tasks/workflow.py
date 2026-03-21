"""
power_panel 装配任务的流程内核。

职责：
  1. 统一封装感知入口（示例图、文件、真实采集、mock）
  2. 构造 HTN 初始状态
  3. 生成计划并提供模拟执行轨迹
"""

from __future__ import annotations

import copy
import json
import math
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# 路径设置
_PLAN_ROOT = Path(__file__).resolve().parent.parent.parent
_HTN_ROOT = Path(__file__).resolve().parent.parent / "htn"
for _path in (str(_PLAN_ROOT), str(_HTN_ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from robot_htn import State, htn

import plan.tasks.operators as _ops  # noqa: F401
import plan.tasks.methods as _mth  # noqa: F401
from plan.tasks.operators import EXPECTED_HOLES


HOME = (0.30, -0.38, 0.15, -math.pi, 0.0, 0.0)
BIN_SPECT = (-0.40, 0.25, 0.25, -math.pi, 0.0, 0.0)
SPEED = 0.20
ACCEL = 0.10

IMAGE_W, IMAGE_H = 1280, 720
X_LEFT_TOP = +0.25
Y_LEFT_TOP = -0.13
X_SPAN = 0.51
Y_SPAN = 0.278
KX = X_SPAN / IMAGE_W
KY = Y_SPAN / IMAGE_H
DX_TCP = -0.0458
DY_TCP = -0.082
DZ_TCP = 0.0

RESULTS_ROOT = Path(__file__).resolve().parent.parent / "results"
YOLO_ROOT = Path(__file__).resolve().parent.parent / "yolo"
DEFAULT_BIN_EXAMPLE = YOLO_ROOT / "step1.png"
DEFAULT_HOME_EXAMPLE = YOLO_ROOT / "step2.png"
DEFAULT_HOME_FAIL_EXAMPLE = YOLO_ROOT / "step2_no.png"

STATE_LABELS = {
    "robot_at": "机器人位置",
    "bin_inspected": "料箱已检查",
    "bin_has_panel": "料箱有 panel",
    "holding_panel": "正在持有 panel",
    "panel_placed": "已放置到背板",
    "placement_verified": "已完成放置验证",
    "placement_ok": "放置验证通过",
    "placement_attempts": "放置验证次数",
    "max_placement_attempts": "最大放置尝试次数",
    "holes_detected": "孔位已识别",
    "missing_holes": "缺失孔位",
    "screws_fastened": "已完成螺丝",
    "screws_available": "剩余螺丝数量",
    "manual_review_required": "需要人工复核",
    "manual_review_reason": "人工复核原因",
    "workflow_status": "流程状态",
}

SUMMARY_KEYS = (
    "robot_at",
    "bin_inspected",
    "bin_has_panel",
    "holding_panel",
    "panel_placed",
    "placement_verified",
    "placement_ok",
    "placement_attempts",
    "max_placement_attempts",
    "holes_detected",
    "missing_holes",
    "screws_fastened",
    "screws_available",
    "manual_review_required",
    "manual_review_reason",
    "workflow_status",
)


@dataclass
class StepResult:
    title: str
    success: bool
    message: str
    raw_path: Optional[Path] = None
    annotated_path: Optional[Path] = None
    data: Dict[str, Any] = field(default_factory=dict)


@dataclass
class WorkflowPerception:
    save_dir: Path
    bin_step: StepResult
    panel_step: StepResult
    screw_step: StepResult
    source_mode: str = "files"

    @property
    def bin_result(self) -> bool:
        return bool(self.bin_step.data.get("found", self.bin_step.success))

    @property
    def panel_result(self) -> bool:
        return bool(self.panel_step.data.get("installed", self.panel_step.success))

    @property
    def screw_holes(self) -> Dict[str, Tuple[float, float]]:
        return dict(self.screw_step.data.get("hole_positions", {}))

    @property
    def missing_holes(self) -> List[str]:
        return list(self.screw_step.data.get("missing_holes", []))


def _json_ready(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, tuple):
        return [_json_ready(item) for item in value]
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    return value


def _serialize_path(path: Optional[Path], base_dir: Path) -> Optional[str]:
    if path is None:
        return None
    path = Path(path)
    try:
        return str(path.relative_to(base_dir))
    except ValueError:
        return str(path)


def _deserialize_path(path_text: Optional[str], base_dir: Path) -> Optional[Path]:
    if not path_text:
        return None
    path = Path(path_text)
    if path.is_absolute():
        return path
    return (base_dir / path).resolve()


def _serialize_step_result(step: StepResult, base_dir: Path) -> Dict[str, Any]:
    return {
        "title": step.title,
        "success": bool(step.success),
        "message": step.message,
        "raw_path": _serialize_path(step.raw_path, base_dir),
        "annotated_path": _serialize_path(step.annotated_path, base_dir),
        "data": _json_ready(step.data),
    }


def _deserialize_step_result(payload: Dict[str, Any], base_dir: Path) -> StepResult:
    data = dict(payload.get("data", {}))
    if "hole_positions" in data:
        data["hole_positions"] = {
            hole: tuple(position)
            for hole, position in dict(data["hole_positions"]).items()
        }
    return StepResult(
        title=str(payload.get("title", "")),
        success=bool(payload.get("success", False)),
        message=str(payload.get("message", "")),
        raw_path=_deserialize_path(payload.get("raw_path"), base_dir),
        annotated_path=_deserialize_path(payload.get("annotated_path"), base_dir),
        data=data,
    )


def save_perception_bundle(
    perception: WorkflowPerception,
    output_path: Optional[Path] = None,
) -> Path:
    output_path = Path(output_path) if output_path else perception.save_dir / "perception_result.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    base_dir = output_path.parent
    payload = {
        "save_dir": _serialize_path(perception.save_dir, base_dir),
        "source_mode": perception.source_mode,
        "bin_step": _serialize_step_result(perception.bin_step, base_dir),
        "panel_step": _serialize_step_result(perception.panel_step, base_dir),
        "screw_step": _serialize_step_result(perception.screw_step, base_dir),
    }
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return output_path


def load_perception_bundle(bundle_path: Path) -> WorkflowPerception:
    bundle_path = Path(bundle_path).resolve()
    payload = json.loads(bundle_path.read_text(encoding="utf-8"))
    base_dir = bundle_path.parent
    save_dir = _deserialize_path(payload.get("save_dir"), base_dir) or base_dir
    return WorkflowPerception(
        save_dir=save_dir,
        bin_step=_deserialize_step_result(payload.get("bin_step", {}), base_dir),
        panel_step=_deserialize_step_result(payload.get("panel_step", {}), base_dir),
        screw_step=_deserialize_step_result(payload.get("screw_step", {}), base_dir),
        source_mode=str(payload.get("source_mode", "imported")),
    )


def create_results_dir(base_dir: Optional[Path] = None) -> Path:
    root = Path(base_dir) if base_dir else RESULTS_ROOT
    save_dir = root / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    save_dir.mkdir(parents=True, exist_ok=True)
    return save_dir


def get_template_hole_positions() -> Dict[str, Tuple[float, float]]:
    from plan.perception.screw_detector import (
        _GROUP_TO_LETTER,
        _TEMPLATE_PATH,
        _load_template_groups,
    )

    positions: Dict[str, Tuple[float, float]] = {}
    for group_id, center in _load_template_groups(_TEMPLATE_PATH):
        letter = _GROUP_TO_LETTER.get(group_id)
        if letter:
            positions[f"hole_{letter}"] = center
    return positions


def default_hole_positions(selected_holes: Iterable[str]) -> Dict[str, Tuple[float, float]]:
    template_positions = get_template_hole_positions()
    return {
        hole: tuple(template_positions[hole])
        for hole in EXPECTED_HOLES
        if hole in selected_holes and hole in template_positions
    }


def parse_panel_sequence(text: str, fallback: bool) -> List[bool]:
    raw = text.strip()
    if not raw:
        return [bool(fallback)]

    normalized_text = raw.replace("，", ",").replace("；", ",").replace(";", ",")
    mapping = {
        "成功": True,
        "通过": True,
        "是": True,
        "有": True,
        "1": True,
        "true": True,
        "yes": True,
        "ok": True,
        "失败": False,
        "未通过": False,
        "否": False,
        "无": False,
        "0": False,
        "false": False,
        "no": False,
        "fail": False,
    }

    results: List[bool] = []
    for token in normalized_text.split(","):
        cleaned = token.strip()
        if not cleaned:
            continue
        lowered = cleaned.lower()
        if cleaned in mapping:
            results.append(mapping[cleaned])
            continue
        if lowered in mapping:
            results.append(mapping[lowered])
            continue
        raise ValueError(f"无法识别放置验证序列项: {cleaned}")

    if not results:
        return [bool(fallback)]
    return results


def format_action(action: Tuple[Any, ...]) -> str:
    name = str(action[0])
    if len(action) == 1:
        return f"{name}()"
    args = ", ".join(str(arg) for arg in action[1:])
    return f"{name}({args})"


def format_state_summary(state: State) -> str:
    lines: List[str] = []
    for key in SUMMARY_KEYS:
        if hasattr(state, key):
            label = STATE_LABELS.get(key, key)
            value = getattr(state, key)
            lines.append(f"{label}: {value}")
    return "\n".join(lines)


def format_plan(plan: Optional[List[Tuple[Any, ...]]]) -> str:
    if plan is False or plan is None:
        return "未找到可执行计划"
    if len(plan) == 0:
        return "计划为空：当前状态已满足目标"
    lines = [f"共 {len(plan)} 步："]
    for index, action in enumerate(plan, start=1):
        lines.append(f"{index:02d}. {format_action(action)}")
    return "\n".join(lines)


def _prepare_stage_image(
    stage_dir: Path,
    filename: str,
    image: Optional[np.ndarray] = None,
    source_path: Optional[Path] = None,
) -> Path:
    stage_dir.mkdir(parents=True, exist_ok=True)
    target_path = stage_dir / filename

    if image is not None:
        if not cv2.imwrite(str(target_path), image):
            raise RuntimeError(f"保存图像失败: {target_path}")
        return target_path

    if source_path is None:
        raise ValueError("必须提供 image 或 source_path")

    source_path = Path(source_path)
    frame = cv2.imread(str(source_path))
    if frame is None:
        raise FileNotFoundError(f"无法读取图像: {source_path}")
    if not cv2.imwrite(str(target_path), frame):
        raise RuntimeError(f"保存图像失败: {target_path}")
    return target_path


def _best_confidence(detections: Sequence[Sequence[Any]]) -> Optional[float]:
    confidences: List[float] = []
    for detection in detections:
        if len(detection) > 4:
            try:
                confidences.append(float(detection[4]))
            except (TypeError, ValueError):
                continue
    if not confidences:
        return None
    return max(confidences)


def _run_bin_detection(raw_path: Path, stage_dir: Path) -> StepResult:
    from plan.perception.bin_detector import BinDetector

    result = BinDetector().detect(raw_path, save_dir=None)
    annotated_path = stage_dir / "step1_annotated.png"
    cv2.imwrite(str(annotated_path), result["annotated"])
    best_conf = _best_confidence(result["detections"])

    message = "料箱中检测到 power_panel" if result["found"] else "料箱中未检测到 power_panel"
    if best_conf is not None:
        message += f"（最高置信度 {best_conf:.2f}）"

    return StepResult(
        title="料箱检测",
        success=bool(result["found"]),
        message=message,
        raw_path=raw_path,
        annotated_path=annotated_path,
        data={
            "found": bool(result["found"]),
            "count": int(result["count"]),
            "detections": result["detections"],
            "best_confidence": best_conf,
        },
    )


def _run_panel_detection(raw_path: Path, stage_dir: Path) -> StepResult:
    from plan.perception.panel_detector import PanelDetector

    result = PanelDetector().detect(raw_path, save_dir=None)
    annotated_path = stage_dir / "step2_annotated.png"
    cv2.imwrite(str(annotated_path), result["annotated"])
    best_conf = _best_confidence(result["detections"])

    message = "检测到 power_panel 已安装到位" if result["installed"] else "未确认 power_panel 安装到位"
    if best_conf is not None:
        message += f"（最高置信度 {best_conf:.2f}）"

    return StepResult(
        title="放置验证",
        success=bool(result["installed"]),
        message=message,
        raw_path=raw_path,
        annotated_path=annotated_path,
        data={
            "installed": bool(result["installed"]),
            "count": int(result["count"]),
            "detections": result["detections"],
            "best_confidence": best_conf,
        },
    )


def _run_screw_detection(raw_path: Path, stage_dir: Path) -> StepResult:
    from plan.perception.screw_detector import ScrewDetector

    result = ScrewDetector().detect(raw_path, save_dir=None)
    annotated_path = stage_dir / "step3_annotated.png"
    cv2.imwrite(str(annotated_path), result["annotated"])

    hole_positions = {
        hole_name: tuple(info["position"])
        for hole_name, info in result["holes"].items()
        if info.get("position") is not None
    }
    hole_statuses = {
        hole_name: info.get("status")
        for hole_name, info in result["holes"].items()
        if hole_name in hole_positions
    }
    missing_holes = [hole for hole in EXPECTED_HOLES if hole not in hole_positions]

    detected_count = len(hole_positions)
    message = f"已识别 {detected_count}/{len(EXPECTED_HOLES)} 个孔位"
    if missing_holes:
        message += "，缺失: " + ", ".join(missing_holes)
    else:
        message += "，孔位识别完整"

    return StepResult(
        title="螺丝孔检测",
        success=not missing_holes,
        message=message,
        raw_path=raw_path,
        annotated_path=annotated_path,
        data={
            "hole_positions": hole_positions,
            "hole_statuses": hole_statuses,
            "missing_holes": missing_holes,
            "all_ready": bool(result["all_ready"]),
        },
    )


def run_perception_from_files(
    bin_image_path: Path,
    home_image_path: Path,
    save_dir: Optional[Path] = None,
) -> WorkflowPerception:
    save_dir = save_dir or create_results_dir()
    bin_stage_dir = save_dir / "step1"
    home_stage_dir = save_dir / "step2"
    screw_stage_dir = save_dir / "step3"

    bin_raw = _prepare_stage_image(bin_stage_dir, "step1_raw.png", source_path=Path(bin_image_path))
    panel_raw = _prepare_stage_image(home_stage_dir, "step2_raw.png", source_path=Path(home_image_path))
    screw_raw = _prepare_stage_image(screw_stage_dir, "step3_raw.png", source_path=Path(home_image_path))

    perception = WorkflowPerception(
        save_dir=save_dir,
        bin_step=_run_bin_detection(bin_raw, bin_stage_dir),
        panel_step=_run_panel_detection(panel_raw, home_stage_dir),
        screw_step=_run_screw_detection(screw_raw, screw_stage_dir),
        source_mode="files",
    )
    save_perception_bundle(perception)
    return perception


def run_perception_from_arrays(
    bin_image: np.ndarray,
    home_image: np.ndarray,
    save_dir: Optional[Path] = None,
) -> WorkflowPerception:
    save_dir = save_dir or create_results_dir()
    bin_stage_dir = save_dir / "step1"
    home_stage_dir = save_dir / "step2"
    screw_stage_dir = save_dir / "step3"

    bin_raw = _prepare_stage_image(bin_stage_dir, "step1_raw.png", image=bin_image)
    panel_raw = _prepare_stage_image(home_stage_dir, "step2_raw.png", image=home_image)
    screw_raw = _prepare_stage_image(screw_stage_dir, "step3_raw.png", image=home_image)

    perception = WorkflowPerception(
        save_dir=save_dir,
        bin_step=_run_bin_detection(bin_raw, bin_stage_dir),
        panel_step=_run_panel_detection(panel_raw, home_stage_dir),
        screw_step=_run_screw_detection(screw_raw, screw_stage_dir),
        source_mode="arrays",
    )
    save_perception_bundle(perception)
    return perception


def run_example_perception(
    save_dir: Optional[Path] = None,
    placement_ok: bool = True,
) -> WorkflowPerception:
    home_image = DEFAULT_HOME_EXAMPLE if placement_ok else DEFAULT_HOME_FAIL_EXAMPLE
    return run_perception_from_files(
        DEFAULT_BIN_EXAMPLE,
        home_image,
        save_dir=save_dir,
    )


def build_mock_perception(
    save_dir: Optional[Path] = None,
    bin_result: bool = True,
    panel_result: bool = True,
    holes: Optional[Iterable[str]] = None,
) -> WorkflowPerception:
    save_dir = save_dir or create_results_dir()
    selected_holes = set(holes or EXPECTED_HOLES)
    hole_positions = default_hole_positions(selected_holes)
    missing_holes = [hole for hole in EXPECTED_HOLES if hole not in hole_positions]

    perception = WorkflowPerception(
        save_dir=save_dir,
        bin_step=StepResult(
            title="料箱检测",
            success=bin_result,
            message="mock：料箱中有 power_panel" if bin_result else "mock：料箱中无 power_panel",
            data={"found": bin_result, "count": int(bin_result), "detections": []},
        ),
        panel_step=StepResult(
            title="放置验证",
            success=panel_result,
            message="mock：安装验证通过" if panel_result else "mock：安装验证失败",
            data={"installed": panel_result, "count": int(panel_result), "detections": []},
        ),
        screw_step=StepResult(
            title="螺丝孔检测",
            success=not missing_holes,
            message=(
                "mock：孔位识别完整"
                if not missing_holes
                else "mock：孔位缺失 " + ", ".join(missing_holes)
            ),
            data={
                "hole_positions": hole_positions,
                "hole_statuses": {hole: "zhun" for hole in hole_positions},
                "missing_holes": missing_holes,
                "all_ready": not missing_holes,
            },
        ),
        source_mode="mock",
    )
    save_perception_bundle(perception)
    return perception


def capture_frame() -> np.ndarray:
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    profile = pipeline.start(config)

    try:
        device = profile.get_device()
        color_sensor = next(
            sensor
            for sensor in device.query_sensors()
            if sensor.get_info(rs.camera_info.name) == "RGB Camera"
        )

        if color_sensor.supports(rs.option.enable_auto_exposure):
            color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        if color_sensor.supports(rs.option.enable_auto_white_balance):
            color_sensor.set_option(rs.option.enable_auto_white_balance, 1)

        warmup_start = time.time()
        color_frame = None
        while time.time() - warmup_start < 3.0:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

        if not color_frame:
            raise RuntimeError("未能从 D415 获取彩色帧")

        for _ in range(10):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

        image_rgb = np.asanyarray(color_frame.get_data())
        return cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    finally:
        pipeline.stop()


def _build_live_arm_client(node_name: str = "power_panel_task_node"):
    import rclpy
    from hyy_message.action import MoveXYZW
    from rclpy.action import ActionClient
    from rclpy.node import Node

    class ArmClient(Node):
        def __init__(self):
            super().__init__(node_name)
            self._ac = ActionClient(self, MoveXYZW, "MoveXYZW")
            self.get_logger().info("等待 MoveXYZW action server …")
            self._ac.wait_for_server()
            self.get_logger().info("Action server 已就绪")

        def move(self, pose: Tuple[float, float, float, float, float, float], label: str) -> bool:
            x, y, z, roll, pitch, yaw = pose
            goal = MoveXYZW.Goal()
            goal.positionx = x
            goal.positiony = y
            goal.positionz = z
            goal.roll = roll
            goal.pitch = pitch
            goal.yaw = yaw
            goal.speed = max(min(SPEED, 0.2), 0.001)
            goal.accel = max(min(ACCEL, 0.2), 0.001)

            self.get_logger().info(f"[{label}] → ({x:.3f}, {y:.3f}, {z:.3f})")
            future = self._ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"[{label}] Goal 被拒绝")
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            ok = result_future.result().result.result == "MoveXYZW:SUCCESS"
            if ok:
                self.get_logger().info(f"[{label}] ✓ 到位")
            else:
                self.get_logger().error(f"[{label}] ✗ 失败")
            return ok

    return ArmClient()


def collect_live_perception(save_dir: Optional[Path] = None) -> WorkflowPerception:
    import rclpy

    save_dir = save_dir or create_results_dir()
    did_init = False
    if not rclpy.ok():
        rclpy.init()
        did_init = True
    arm = _build_live_arm_client("power_panel_task_node")

    try:
        if not arm.move(BIN_SPECT, "BIN_SPECT"):
            raise RuntimeError("移动到 BIN_SPECT 失败")
        bin_image = capture_frame()

        if not arm.move(HOME, "HOME"):
            raise RuntimeError("移动到 HOME 失败")
        home_image = capture_frame()

        perception = run_perception_from_arrays(bin_image, home_image, save_dir=save_dir)
        perception.source_mode = "live"
        save_perception_bundle(perception)
        return perception
    finally:
        arm.destroy_node()
        if did_init:
            rclpy.shutdown()


def compute_hole_poses(screw_holes: Dict[str, Tuple[float, float]]) -> Dict[str, Tuple[float, float, float, float, float, float]]:
    poses: Dict[str, Tuple[float, float, float, float, float, float]] = {}
    for label, pos in screw_holes.items():
        if pos is None:
            continue
        u, v = pos
        x_hole_cam = X_LEFT_TOP - u * KX
        y_hole_cam = Y_LEFT_TOP + v * KY
        x_target = HOME[0] + DX_TCP + x_hole_cam
        y_target = HOME[1] + DY_TCP + y_hole_cam
        z_target = HOME[2] + DZ_TCP
        poses[label] = (x_target, y_target, z_target, HOME[3], HOME[4], HOME[5])
    return poses


def execute_live_hole_visit(
    screw_holes: Dict[str, Tuple[float, float]],
    dwell_seconds: float = 3.0,
    return_home: bool = True,
    report_hook: Optional[Callable[[str], None]] = None,
) -> List[Dict[str, Any]]:
    import rclpy

    trace: List[Dict[str, Any]] = []
    hole_poses = compute_hole_poses(screw_holes)
    order = [hole for hole in EXPECTED_HOLES if hole in hole_poses]

    def report(message: str) -> None:
        if report_hook is not None:
            report_hook(message)

    if not order:
        return trace

    did_init = False
    if not rclpy.ok():
        rclpy.init()
        did_init = True

    arm = _build_live_arm_client("power_panel_gui_control_node")
    step_index = 1

    try:
        report("真实执行：返回 HOME 准备开始孔位巡检。")
        home_ok = arm.move(HOME, "HOME")
        trace.append(
            {
                "step": step_index,
                "action": "move(HOME)",
                "status": "成功" if home_ok else "失败",
                "changes": [f"目标位姿={HOME[:3]}"],
            }
        )
        step_index += 1
        if not home_ok:
            return trace

        for hole in order:
            pose = hole_poses[hole]
            report(f"真实执行：移动到 {hole} 上方。")
            ok = arm.move(pose, hole)
            changes = [
                f"像素坐标={tuple(round(v, 1) for v in screw_holes[hole])}",
                f"TCP=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})",
            ]
            if ok and dwell_seconds > 0:
                time.sleep(dwell_seconds)
                changes.append(f"停留 {dwell_seconds:.1f} 秒")
            trace.append(
                {
                    "step": step_index,
                    "action": f"visit_hole({hole})",
                    "status": "成功" if ok else "失败",
                    "changes": changes,
                }
            )
            step_index += 1

        if return_home:
            report("真实执行：孔位巡检结束，返回 HOME。")
            ok = arm.move(HOME, "HOME")
            trace.append(
                {
                    "step": step_index,
                    "action": "move(HOME)",
                    "status": "成功" if ok else "失败",
                    "changes": [f"目标位姿={HOME[:3]}"],
                }
            )
    finally:
        arm.destroy_node()
        if did_init:
            rclpy.shutdown()


def build_state(
    bin_result: bool,
    panel_result: bool,
    screw_holes: Dict[str, Tuple[float, float]],
    panel_sequence: Optional[Sequence[bool]] = None,
    max_placement_attempts: int = 2,
) -> State:
    sequence = list(panel_sequence) if panel_sequence else [bool(panel_result)]
    holes = {hole: tuple(position) for hole, position in screw_holes.items() if hole in EXPECTED_HOLES}

    state = State("power_panel_install")
    state.robot_at = "home"
    state._perception_bin_result = bool(bin_result)
    state.bin_inspected = False
    state.bin_has_panel = False
    state.holding_panel = False
    state._perception_panel_result = bool(panel_result)
    state._perception_panel_result_sequence = sequence
    state.panel_placed = False
    state.placement_verified = False
    state.placement_ok = False
    state.placement_attempts = 0
    state.max_placement_attempts = max(1, int(max_placement_attempts))
    state._perception_screw_result = holes
    state.holes_detected = False
    state.expected_holes = list(EXPECTED_HOLES)
    state.hole_positions = {}
    state.missing_holes = []
    state.holding_screw = False
    state.screw_aligned_to = None
    state.screw_inserted_at = None
    state.screws_available = 10
    state.screws_fastened = []
    state.manual_review_required = False
    state.manual_review_reason = None
    state.workflow_status = "待规划"
    return state


def build_state_from_perception(
    perception: WorkflowPerception,
    panel_sequence: Optional[Sequence[bool]] = None,
    max_placement_attempts: int = 2,
    overrides: Optional[Dict[str, Any]] = None,
) -> State:
    overrides = overrides or {}
    bin_result = bool(overrides.get("bin_result", perception.bin_result))
    panel_result = bool(overrides.get("panel_result", perception.panel_result))

    override_holes = overrides.get("screw_holes")
    if override_holes is None:
        screw_holes = perception.screw_holes
    else:
        screw_holes = {
            hole: tuple(position)
            for hole, position in dict(override_holes).items()
            if hole in EXPECTED_HOLES
        }

    sequence = list(panel_sequence) if panel_sequence else [panel_result]
    return build_state(
        bin_result=bin_result,
        panel_result=panel_result,
        screw_holes=screw_holes,
        panel_sequence=sequence,
        max_placement_attempts=max_placement_attempts,
    )


def plan_installation(state: State, verbose: int = 0):
    return htn.plan(
        state,
        [("install_power_panel",)],
        htn.get_operators(),
        htn.get_methods(),
        verbose=verbose,
    )


def _capture_summary(state: State) -> Dict[str, Any]:
    snapshot: Dict[str, Any] = {}
    for key in SUMMARY_KEYS:
        if hasattr(state, key):
            snapshot[key] = copy.deepcopy(getattr(state, key))
    return snapshot


def _describe_changes(before: Dict[str, Any], after: Dict[str, Any]) -> List[str]:
    changes: List[str] = []
    for key in SUMMARY_KEYS:
        if before.get(key) != after.get(key):
            label = STATE_LABELS.get(key, key)
            changes.append(f"{label}: {before.get(key)} -> {after.get(key)}")
    return changes


def simulate_plan(
    initial_state: State,
    plan: Optional[List[Tuple[Any, ...]]],
) -> Tuple[State, List[Dict[str, Any]]]:
    operators = htn.get_operators()
    state = initial_state.copy()
    trace: List[Dict[str, Any]] = []

    if not plan:
        return state, trace

    for index, action in enumerate(plan, start=1):
        action_name = action[0]
        operator = operators.get(action_name)
        before = _capture_summary(state)

        if operator is None:
            trace.append(
                {
                    "step": index,
                    "action": format_action(action),
                    "status": "失败",
                    "changes": ["未找到对应 operator"],
                }
            )
            break

        new_state = operator(state.copy(), *action[1:])
        if not new_state:
            trace.append(
                {
                    "step": index,
                    "action": format_action(action),
                    "status": "失败",
                    "changes": ["operator 前置条件不满足"],
                }
            )
            break

        after = _capture_summary(new_state)
        trace.append(
            {
                "step": index,
                "action": format_action(action),
                "status": "成功",
                "changes": _describe_changes(before, after) or ["无关键状态变化"],
            }
        )
        state = new_state

    return state, trace
