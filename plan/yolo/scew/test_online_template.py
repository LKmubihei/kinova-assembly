from ultralytics import YOLO
from pathlib import Path
import json
import math
import cv2

TEMPLATE_JSON = Path(__file__).resolve().parent / "template.json"


def _load_template_hole_groups(template_json: Path) -> list[tuple[int, tuple[float, float]]]:
    if not template_json.exists():
        raise FileNotFoundError(f"未找到模板json: {template_json}")

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
        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0
        centers.append((cx, cy))

    if not centers:
        raise RuntimeError("模板中未找到 Hole_pcb")

    sorted_centers = sorted(centers, key=lambda c: (c[1], c[0]))
    # desired_order = [2, 5, 3, 7, 4, 6, 1]
    desired_order = [3, 5, 2, 7, 1, 6, 4]

    if len(sorted_centers) != len(desired_order):
        raise RuntimeError(
            f"模板孔位数量({len(sorted_centers)})与预期顺序数量({len(desired_order)})不一致"
        )

    print("模板中心点排序结果 (按 y,x):")
    for idx, (cx, cy) in enumerate(sorted_centers, start=1):
        print(f"  {idx}: ({cx:.2f}, {cy:.2f}) -> group {desired_order[idx - 1]}")

    groups = []
    for idx, center in enumerate(sorted_centers):
        groups.append((desired_order[idx], center))

    return groups


def _assign_group_to_box(center: tuple[float, float], groups: list[tuple[int, tuple[float, float]]]) -> int:
    cx, cy = center
    best_id = -1
    best_dist = float("inf")
    for gid, (gx, gy) in groups:
        dist = math.hypot(cx - gx, cy - gy)
        if dist < best_dist:
            best_dist = dist
            best_id = gid
    return best_id


def _save_grouped_labelme(result, image_path: Path, template_groups: list[tuple[int, tuple[float, float]]]):
    if result.boxes is None:
        return None

    shapes = []
    names = result.names
    for box, cls in zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist()):
        x1, y1, x2, y2 = box
        label = names[int(cls)]
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        group_id = None
        if label != "Peg_pcb":
            group_id = _assign_group_to_box((cx, cy), template_groups)

        shapes.append(
            {
                "label": label,
                "points": [[x1, y1], [x2, y2]],
                "group_id": group_id,
                "shape_type": "rectangle",
                "flags": {},
            }
        )

    output = {
        "version": "5.0.0",
        "flags": {},
        "shapes": shapes,
        "imagePath": image_path.name,
        "imageData": None,
        "imageHeight": result.orig_shape[0],
        "imageWidth": result.orig_shape[1],
    }

    out_json = image_path.with_suffix(".group.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(output, f, ensure_ascii=False, indent=2)

    return out_json


def _draw_group_text(image_path: Path, shapes: list[dict], output_path: Path):
    image = cv2.imread(str(image_path))
    if image is None:
        raise FileNotFoundError(f"无法读取图片: {image_path}")

    for shape in shapes:
        group_id = shape.get("group_id")
        if group_id is None:
            continue

        if shape.get("label") == "Peg_pcb":
            continue

        points = shape.get("points", [])
        if len(points) < 2:
            continue

        (x1, y1), (x2, y2) = points
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 220, 255), 2)
        text = f"G{group_id}"
        text_x = x1
        text_y = max(y1 - 6, 10)
        cv2.putText(
            image,
            text,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 220, 255),
            2,
            cv2.LINE_AA,
        )

    cv2.imwrite(str(output_path), image)
    return output_path


# G编号 -> 螺丝字母 映射
_GROUP_TO_LETTER = {1: "A", 2: "B", 3: "C", 4: "D", 5: "E", 6: "F", 7: "G"}

# zhun/buzhun 检测类名（与模型 names 一致）
_SCREW_STATUS_LABELS = {"zhun", "buzhun"}


def _summarize_hole_status(shapes: list[dict]) -> dict[str, dict]:
    """
    从分组 shapes 中提取每个螺丝孔的位置和螺丝状态。

    返回:
        {
          "hole_A": {"position": (cx, cy), "status": "zhun" | "buzhun" | None},
          ...
        }
    """
    holes: dict[str, dict] = {}

    for shape in shapes:
        label = shape.get("label", "")
        group_id = shape.get("group_id")

        # 跳过凸包及未分组目标
        if label == "Peg_pcb" or group_id is None:
            continue

        letter = _GROUP_TO_LETTER.get(group_id)
        if letter is None:
            continue

        hole_name = f"hole_{letter}"
        if hole_name not in holes:
            holes[hole_name] = {"position": None, "status": None}

        points = shape.get("points", [])
        if len(points) >= 2:
            (x1, y1), (x2, y2) = points[0], points[1]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
        else:
            cx, cy = None, None

        if label == "Hole_pcb":
            if holes[hole_name]["position"] is None and cx is not None:
                holes[hole_name]["position"] = (cx, cy)
        elif label in _SCREW_STATUS_LABELS:
            holes[hole_name]["status"] = label
            # 如果位置尚未由 Hole_pcb 填充，用 zhun/buzhun 框中心作为备用位置
            if holes[hole_name]["position"] is None and cx is not None:
                holes[hole_name]["position"] = (cx, cy)

    return holes


def _print_hole_summary(holes: dict[str, dict]):
    print("\n===== 螺丝孔检测结果 =====")
    for hole_name in sorted(holes.keys()):
        info = holes[hole_name]
        pos = info["position"]
        status = info["status"] if info["status"] is not None else "未检测到"
        pos_str = f"({pos[0]:.1f}, {pos[1]:.1f})" if pos else "未知"
        print(f"  {hole_name}: position={pos_str}, status={status}")
    print("==========================\n")


def predict_single_image(image_path: str):
    """单张图片预测。"""
    best_weight = Path("/data/taosen/runs/detect/pcb_backplane_train/yolov11_detect4/weights/best.pt")
    image_path = Path(image_path)

    if not best_weight.exists():
        raise FileNotFoundError(f"未找到训练权重: {best_weight}")
    if not image_path.exists():
        raise FileNotFoundError(f"未找到图片: {image_path}")

    model = YOLO(str(best_weight))
    pred_results = model.predict(
        source=str(image_path),
        save=True,
        imgsz=1280,
    )

    template_groups = _load_template_hole_groups(TEMPLATE_JSON)
    out_json = _save_grouped_labelme(pred_results[0], image_path, template_groups)
    annotated_path = None
    if out_json is not None:
        with open(out_json, "r", encoding="utf-8") as f:
            grouped_data = json.load(f)
        annotated_path = image_path.with_suffix(".group.png")
        _draw_group_text(image_path, grouped_data.get("shapes", []), annotated_path)

        # 输出每个螺丝孔的汇总结果
        holes = _summarize_hole_status(grouped_data.get("shapes", []))
        _print_hole_summary(holes)

    print("单图预测完成，结果已保存到 runs/detect/predict*")
    if out_json is not None:
        print(f"已生成分组标注: {out_json}")
    if annotated_path is not None:
        print(f"已生成带group文本的图片: {annotated_path}")

    return pred_results


# def predict_stream(source: str | int = 0):
#     """在线/视频流检测（如摄像头或视频流地址）。"""
#     best_weight = Path("/data/taosen/runs/detect/pcb_backplane_train/yolov11_detect4/weights/best.pt")

#     if not best_weight.exists():
#         raise FileNotFoundError(f"未找到训练权重: {best_weight}")

#     model = YOLO(str(best_weight))
#     model.predict(
#         source=source,
#         stream=True,
#         show=True,
#         imgsz=1280,
#     )



if __name__ == "__main__":
    # train_detect()
    # test_detect()
    predict_single_image("/data/taosen/data/screw_buzhun_2/yolo_dataset/images/val/013.png")
    # predict_single_image("/data/taosen/data/screw_buzhun_2/yolo_dataset/online_image/online1.png")
    # predict_stream(0)
