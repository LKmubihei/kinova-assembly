"""
Step 3 - 螺丝孔检测
检测背板上的螺丝孔位置和状态，使用 scew 模型 + template.json 分组
"""

import json
import math
from pathlib import Path
from ultralytics import YOLO

_SCREW_DIR = Path(__file__).parent.parent / "yolo/scew"
_MODEL_PATH = _SCREW_DIR / "best.pt"
_TEMPLATE_PATH = _SCREW_DIR / "template.json"

# group_id -> 螺丝字母
_GROUP_TO_LETTER = {1: "A", 2: "B", 3: "C", 4: "D", 5: "E", 6: "F", 7: "G"}
_SCREW_STATUS_LABELS = {"zhun", "buzhun"}


def _load_template_groups(template_json: Path) -> list:
    with open(template_json, "r", encoding="utf-8") as f:
        data = json.load(f)

    centers = []
    for shape in data.get("shapes", []):
        if shape.get("label") != "Hole_pcb":
            continue
        points = shape.get("points", [])
        if len(points) < 2:
            continue
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        centers.append(((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0))

    if not centers:
        raise RuntimeError("模板中未找到 Hole_pcb")

    sorted_centers = sorted(centers, key=lambda c: (c[1], c[0]))
    desired_order = [3, 5, 2, 7, 1, 6, 4]

    if len(sorted_centers) != len(desired_order):
        raise RuntimeError(
            f"模板孔位数量({len(sorted_centers)})与预期({len(desired_order)})不一致"
        )

    return [(desired_order[i], sorted_centers[i]) for i in range(len(sorted_centers))]


def _assign_group(center: tuple, groups: list) -> int:
    cx, cy = center
    best_id, best_dist = -1, float("inf")
    for gid, (gx, gy) in groups:
        d = math.hypot(cx - gx, cy - gy)
        if d < best_dist:
            best_dist, best_id = d, gid
    return best_id


class ScrewDetector:
    def __init__(self, model_path=None, template_path=None, conf=0.25):
        self.model = YOLO(str(model_path or _MODEL_PATH))
        self.template_path = Path(template_path or _TEMPLATE_PATH)
        self.conf = conf
        self._groups = None  # lazy load

    def _get_groups(self):
        if self._groups is None:
            self._groups = _load_template_groups(self.template_path)
        return self._groups

    def detect(self, image_path, save_dir=None) -> dict:
        """
        检测螺丝孔位置和状态。

        Args:
            image_path: 输入图像路径
            save_dir:   若指定，保存原图和标注图到该目录

        Returns:
            {
                "holes": {
                    "hole_A": {"position": (cx, cy), "status": "zhun"|"buzhun"|None},
                    ...
                },
                "all_ready": bool,
                "annotated": np.ndarray
            }
        """
        import cv2
        results = self.model(str(image_path), conf=self.conf, verbose=False)
        groups = self._get_groups()
        annotated = results[0].plot()

        holes = {}
        r = results[0]
        if r.boxes is None:
            return {"holes": holes, "all_ready": False, "annotated": annotated}

        names = r.names
        for box, cls in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist()):
            x1, y1, x2, y2 = box
            label = names[int(cls)]
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            if label == "Peg_pcb":
                continue

            gid = _assign_group((cx, cy), groups)
            letter = _GROUP_TO_LETTER.get(gid)
            if letter is None:
                continue

            hole_name = f"hole_{letter}"
            if hole_name not in holes:
                holes[hole_name] = {"position": None, "status": None}

            if label == "Hole_pcb" and holes[hole_name]["position"] is None:
                holes[hole_name]["position"] = (cx, cy)
            elif label in _SCREW_STATUS_LABELS:
                holes[hole_name]["status"] = label
                if holes[hole_name]["position"] is None:
                    holes[hole_name]["position"] = (cx, cy)

        if save_dir is not None:
            save_dir = Path(save_dir)
            save_dir.mkdir(parents=True, exist_ok=True)
            src = Path(image_path)
            cv2.imwrite(str(save_dir / f"{src.stem}_raw.png"), cv2.imread(str(src)))
            cv2.imwrite(str(save_dir / f"{src.stem}_annotated.png"), annotated)

        all_ready = (
            len(holes) > 0
            and all(h["status"] == "zhun" for h in holes.values())
        )
        return {"holes": holes, "all_ready": all_ready, "annotated": annotated}


if __name__ == "__main__":
    import sys
    img = sys.argv[1] if len(sys.argv) > 1 else str(
        Path(__file__).parent.parent / "yolo/step3.png"
    )
    det = ScrewDetector()
    result = det.detect(img)
    for name, info in sorted(result["holes"].items()):
        pos = info["position"]
        pos_str = f"({pos[0]:.1f}, {pos[1]:.1f})" if pos else "未知"
        print(f"  {name}: pos={pos_str}  status={info['status']}")
    print(f"all_ready={result['all_ready']}")
