#!/usr/bin/env python3
"""
物料箱PCB检测 - 测试脚本
用于验证模型效果，统计PCB数量
"""

from pathlib import Path
from ultralytics import YOLO
import json
import cv2
import argparse
import torch

# ==================== 配置区域 ====================
# 获取当前脚本所在目录的绝对路径
SCRIPT_DIR = Path(__file__).resolve().parent
# bin_pcb_detection 根目录
BIN_ROOT = SCRIPT_DIR.parent

# 数据集配置文件
DATA_YAML = BIN_ROOT / "bin_data.yaml"

# 模型目录
MODELS_DIR = BIN_ROOT / "models"

# 结果保存目录
RESULTS_DIR = BIN_ROOT / "results"

# 默认输出目录
DEFAULT_OUTPUT_DIR = RESULTS_DIR / "test_results"

# 图片尺寸（与训练保持一致）
IMGSZ = 800
# =================================================


def find_best_weight() -> Path:
    """查找最佳模型权重"""
    # 优先使用 models 目录下的 bin_model.pt
    model_path = MODELS_DIR / "bin_model.pt"
    if model_path.exists():
        return model_path
    
    # 尝试查找最新的训练结果
    weights = list(RESULTS_DIR.glob("*/weights/bin_model.pt"))
    if weights:
        return sorted(weights)[-1]
    
    # 尝试查找 best.pt（兼容旧版本）
    weights = list(RESULTS_DIR.glob("*/weights/best.pt"))
    if weights:
        best_weight = sorted(weights)[-1]
        print(f"⚠️ 使用兼容模式: {best_weight}")
        return best_weight
    
    raise FileNotFoundError(
        f"未找到训练权重，请先训练模型\n"
        f"尝试搜索路径: {MODELS_DIR} 和 {RESULTS_DIR}"
    )


def predict_single_image(image_path: str | Path, model: YOLO, conf: float = 0.25):
    """
    预测单张图片，返回检测结果
    """
    image_path = Path(image_path)
    if not image_path.exists():
        raise FileNotFoundError(f"未找到图片: {image_path}")
    
    results = model.predict(
        source=str(image_path),
        save=False,
        imgsz=IMGSZ,
        conf=conf,
    )[0]
    
    # 解析检测结果
    detections = []
    if results.boxes is not None:
        for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
            x1, y1, x2, y2 = box.tolist()
            detections.append({
                "bbox": [x1, y1, x2, y2],
                "class": int(cls),
                "confidence": float(conf),
            })
    
    return detections, results


def draw_detections(image_path: Path, detections: list, output_path: Path):
    """
    在图片上绘制检测框并保存
    """
    image = cv2.imread(str(image_path))
    if image is None:
        raise FileNotFoundError(f"无法读取图片: {image_path}")
    
    for det in detections:
        x1, y1, x2, y2 = map(int, det["bbox"])
        conf = det["confidence"]
        
        # 绘制矩形框
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # 绘制标签
        label = f"pcb {conf:.2f}"
        (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(image, (x1, y1 - h - 6), (x1 + w, y1), (0, 255, 0), -1)
        cv2.putText(image, label, (x1, y1 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    
    # 在图片左上角显示统计信息
    cv2.putText(image, f"Total PCB: {len(detections)}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    cv2.imwrite(str(output_path), image)
    return output_path


def test_on_val():
    """在验证集上测试模型"""
    print("=" * 60)
    print("📊 物料箱PCB检测 - 验证集测试")
    print("=" * 60)
    print(f"项目根目录: {BIN_ROOT}")
    print(f"配置文件: {DATA_YAML}")
    
    # 检查配置文件
    if not DATA_YAML.exists():
        raise FileNotFoundError(f"配置文件不存在: {DATA_YAML}")
    
    # 查找模型
    try:
        weight_path = find_best_weight()
        print(f"使用模型: {weight_path}")
    except FileNotFoundError as e:
        print(f"❌ 错误: {e}")
        return
    
    # 加载模型
    model = YOLO(str(weight_path))
    
    # 验证集评估
    print("\n运行验证集评估...")
    val_results = model.val(
        data=str(DATA_YAML),
        split="val",
        imgsz=IMGSZ,
        device=0 if torch.cuda.is_available() else 'cpu',
    )
    
    # 获取指标
    metrics = {
        "precision": float(val_results.box.mp),
        "recall": float(val_results.box.mr),
        "mAP50": float(val_results.box.map50),
        "mAP50-95": float(val_results.box.map),
    }
    
    print("\n📊 验证集指标:")
    for k, v in metrics.items():
        print(f"  {k}: {v:.4f}")
    
    return metrics


def test_on_images(image_dir: str | Path, output_dir: str | Path = None):
    """
    测试整个文件夹的图片
    """
    image_dir = Path(image_dir)
    if not image_dir.exists():
        raise FileNotFoundError(f"图片目录不存在: {image_dir}")
    
    # 设置输出目录
    if output_dir is None:
        output_dir = DEFAULT_OUTPUT_DIR / image_dir.name
    else:
        output_dir = Path(output_dir)
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("📸 批量图片检测")
    print("=" * 60)
    print(f"图片目录: {image_dir}")
    print(f"输出目录: {output_dir}")
    
    # 查找模型
    try:
        weight_path = find_best_weight()
        print(f"使用模型: {weight_path}")
    except FileNotFoundError as e:
        print(f"❌ 错误: {e}")
        return
    
    # 加载模型
    model = YOLO(str(weight_path))
    
    # 获取所有图片
    image_files = []
    for ext in ["*.png", "*.jpg", "*.jpeg", "*.bmp"]:
        image_files.extend(image_dir.glob(ext))
    image_files = sorted(image_files)
    
    print(f"找到 {len(image_files)} 张图片")
    
    # 统计结果
    results_summary = []
    total_pcbs = 0
    empty_boxes = 0
    
    for img_path in image_files:
        try:
            # 预测
            detections, _ = predict_single_image(img_path, model)
            
            # 统计
            pcb_count = len(detections)
            total_pcbs += pcb_count
            if pcb_count == 0:
                empty_boxes += 1
            
            # 保存带标注的图片
            output_img = output_dir / img_path.name
            draw_detections(img_path, detections, output_img)
            
            # 保存检测结果JSON
            result_json = output_dir / f"{img_path.stem}.json"
            with open(result_json, "w", encoding="utf-8") as f:
                json.dump({
                    "image": img_path.name,
                    "pcb_count": pcb_count,
                    "detections": detections,
                }, f, indent=2)
            
            status = "✅ 有PCB" if pcb_count > 0 else "⬜ 空料箱"
            print(f"{img_path.name}: {status} (检测到 {pcb_count} 个PCB)")
            
            results_summary.append({
                "image": img_path.name,
                "pcb_count": pcb_count,
                "detections": detections,
            })
            
        except Exception as e:
            print(f"❌ 错误处理 {img_path.name}: {e}")
    
    # 打印统计
    print("\n" + "=" * 60)
    print("📊 测试结果统计")
    print("=" * 60)
    print(f"总图片数: {len(image_files)}")
    print(f"空料箱: {empty_boxes} 张")
    print(f"有PCB: {len(image_files) - empty_boxes} 张")
    print(f"总PCB数量: {total_pcbs}")
    if len(image_files) - empty_boxes > 0:
        print(f"平均每箱PCB数: {total_pcbs/(len(image_files) - empty_boxes):.2f}")
    
    # 保存汇总结果
    summary_file = output_dir / "summary.json"
    with open(summary_file, "w", encoding="utf-8") as f:
        json.dump({
            "total_images": len(image_files),
            "empty_boxes": empty_boxes,
            "total_pcbs": total_pcbs,
            "results": results_summary,
        }, f, indent=2)
    
    print(f"\n📁 详细结果已保存到: {output_dir}")
    return results_summary


def predict_stream(source: str | int = 0):
    """
    摄像头/视频流实时检测
    """
    print("=" * 60)
    print("🎥 摄像头实时检测")
    print("=" * 60)
    
    try:
        weight_path = find_best_weight()
        print(f"使用模型: {weight_path}")
    except FileNotFoundError as e:
        print(f"❌ 错误: {e}")
        return
    
    model = YOLO(str(weight_path))
    
    print(f"开始实时检测，按 'q' 退出...")
    model.predict(
        source=source,
        stream=True,
        show=True,
        imgsz=IMGSZ,
        conf=0.25,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="物料箱PCB检测测试脚本")
    parser.add_argument("--mode", type=str, default="val", 
                        choices=["val", "images", "stream"],
                        help="测试模式: val(验证集), images(图片文件夹), stream(摄像头)")
    parser.add_argument("--source", type=str, default=None,
                        help="图片目录或摄像头ID (默认: 验证集或摄像头0)")
    parser.add_argument("--output", type=str, default=None,
                        help="输出目录 (默认: results/test_results)")
    
    args = parser.parse_args()
    
    try:
        if args.mode == "val":
            # 在验证集上测试
            test_on_val()
            
        elif args.mode == "images":
            # 测试图片文件夹
            if args.source is None:
                # 默认使用验证集
                source = BIN_ROOT / "data" / "images" / "val"
            else:
                source = args.source
            test_on_images(source, args.output)
            
        elif args.mode == "stream":
            # 摄像头实时检测
            source = args.source if args.source is not None else 0
            predict_stream(source)
            
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"错误: {e}")