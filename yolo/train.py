from pathlib import Path
from ultralytics import YOLO

BASE_DIR = Path(__file__).resolve().parent
DATA_YAML = BASE_DIR / "pcb_backplane.yaml"


def train_detect():
    """训练 YOLOv11 目标检测模型。"""
    model = YOLO("yolo11n.pt")

    if not DATA_YAML.exists():
        raise FileNotFoundError(f"未找到数据集配置文件: {DATA_YAML}")

    results = model.train(
        data=str(DATA_YAML),
        epochs=100,
        imgsz=1280,
        device=0,
        project="pcb_backplane_train",
        name="yolov11_detect",
        batch=32,
    )

    print(f"训练完成，结果保存在: {results.save_dir}")


def test_detect():
    """加载 best.pt，对 val 做预测和指标评估。"""
    best_weight = BASE_DIR / "best.pt"
    dataset_yaml = DATA_YAML
    val_images = Path("/data/taosen/data/screw_buzhun_2/yolo_dataset/images/val")

    if not best_weight.exists():
        raise FileNotFoundError(f"未找到训练权重: {best_weight}")
    if not dataset_yaml.exists():
        raise FileNotFoundError(f"未找到数据集配置文件: {dataset_yaml}")
    if not val_images.exists():
        raise FileNotFoundError(f"未找到验证集目录: {val_images}")

    model = YOLO(str(best_weight))

    pred_results = model.predict(
        source=str(val_images),
        save=True,
        imgsz=1280,
    )

    val_results = model.val(
        data=str(dataset_yaml),
        split="val",
        imgsz=1280,
    )

    results_dict = getattr(val_results, "results_dict", {}) or {}
    metrics = {
        "metrics/precision(B)": float(results_dict.get("metrics/precision(B)", 0.0)),
        "metrics/recall(B)": float(results_dict.get("metrics/recall(B)", 0.0)),
        "metrics/mAP50(B)": float(results_dict.get("metrics/mAP50(B)", 0.0)),
        "metrics/mAP50-95(B)": float(results_dict.get("metrics/mAP50-95(B)", 0.0)),
    }

    print(f"已加载权重: {best_weight}")
    print("验证集预测完成，结果已保存到 runs/detect/predict*")
    print("验证集指标:", metrics)

    return {
        "pred_results": pred_results,
        "metrics": metrics,
    }

def predict_single_image(image_path: str):
    """单张图片预测。"""
    best_weight = BASE_DIR / "best.pt"
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

    print(f"单图预测完成，结果已保存到 runs/detect/predict*")
    return pred_results


def predict_stream(source: str | int = 0):
    """在线/视频流检测（如摄像头或视频流地址）。"""
    best_weight = BASE_DIR / "best.pt"

    if not best_weight.exists():
        raise FileNotFoundError(f"未找到训练权重: {best_weight}")

    model = YOLO(str(best_weight))
    model.predict(
        source=source,
        stream=True,
        show=True,
        imgsz=1280,
    )


if __name__ == "__main__":
    # train_detect()
    # test_detect()
    # 单张图片测试示例：
    predict_single_image(str(BASE_DIR / "capture_at_home.png"))
    # 在线检测示例（摄像头）：
    # predict_stream(0)
    pass
