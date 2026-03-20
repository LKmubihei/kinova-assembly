#!/usr/bin/env python3
"""
物料箱PCB检测 - 计数脚本
检测料箱中的PCB板数量
支持智能摄像头适配
"""

from pathlib import Path
from ultralytics import YOLO
import json
import cv2
import argparse
import time

# ==================== 配置区域 ====================
SCRIPT_DIR = Path(__file__).resolve().parent
# bin_pcb_detection 根目录
BIN_ROOT = SCRIPT_DIR.parent

# 模型目录
MODELS_DIR = BIN_ROOT / "models"

# 结果保存目录
RESULTS_DIR = BIN_ROOT / "results"

# 默认输出目录
DEFAULT_OUTPUT_DIR = RESULTS_DIR / "inference"

# 模型路径（优先使用 bin_model.pt）
BEST_WEIGHT = MODELS_DIR / "bin_model.pt"

# 图片尺寸（必须与训练时一致）
IMGSZ = 800

# 默认置信度阈值
DEFAULT_CONF = 0.25
# =================================================


def find_best_weight() -> Path:
    """查找最佳模型权重"""
    # 优先使用 models 目录下的 bin_model.pt
    if BEST_WEIGHT.exists():
        return BEST_WEIGHT
    
    # 尝试查找最新的训练结果
    weights = list(RESULTS_DIR.glob("*/weights/bin_model.pt"))
    if weights:
        return sorted(weights)[-1]
    
    # 尝试查找 best.pt（兼容旧版本）
    weights = list(RESULTS_DIR.glob("*/weights/best.pt"))
    if weights:
        best_weight = sorted(weights)[-1]
        print(f"⚠️ 使用兼容模型: {best_weight}")
        return best_weight
    
    raise FileNotFoundError(
        f"未找到模型权重，请先训练模型\n"
        f"尝试搜索路径: {MODELS_DIR} 和 {RESULTS_DIR}"
    )


def _determine_status(pcb_count: int) -> tuple[str, str]:
    """
    根据PCB数量判定状态
    返回: (状态描述, 状态码)
    """
    if pcb_count == 0:
        return "空料箱", "empty"
    else:
        return f"有 {pcb_count} 个PCB", "has_pcb"


def _draw_results(image, results, pcb_count: int):
    """
    在图片上绘制检测结果和状态
    图片上显示英文，终端输出保持中文
    """
    # 绘制检测框
    if results.boxes is not None:
        for box, conf in zip(results.boxes.xyxy, results.boxes.conf):
            x1, y1, x2, y2 = map(int, box.tolist())
            
            # 画矩形框
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 写标签（英文）
            label = f"PCB {conf:.2f}"
            cv2.putText(image, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # 根据状态选择颜色和英文文字
    if pcb_count == 0:
        status_text = "Empty Box"
        status_color = (0, 0, 255)  # 红色
    else:
        status_text = f"Has {pcb_count} PCB"
        status_color = (0, 255, 0)  # 绿色
    
    # 添加状态文字（英文）
    cv2.putText(image, f"Status: {status_text}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
    cv2.putText(image, f"Total: {pcb_count}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
    
    return image


def predict_single_image(image_path: str, output_dir: str = None, conf_threshold: float = DEFAULT_CONF):
    """
    单张图片检测并判定状态
    """
    image_path = Path(image_path)
    
    # 查找模型
    try:
        weight_path = find_best_weight()
    except FileNotFoundError as e:
        raise FileNotFoundError(f"{e}\n请先训练模型或检查 {MODELS_DIR} 目录")
    
    if not image_path.exists():
        raise FileNotFoundError(f"未找到图片: {image_path}")
    
    # 加载模型
    model = YOLO(str(weight_path))
    print(f"已加载模型: {weight_path}")
    print(f"置信度阈值: {conf_threshold}")
    
    # 预测
    results = model.predict(
        source=str(image_path),
        save=False,
        imgsz=IMGSZ,
        conf=conf_threshold,
    )[0]
    
    # 统计PCB数量
    if results.boxes is not None:
        pcb_count = len(results.boxes)
        confidences = [float(conf) for conf in results.boxes.conf]
    else:
        pcb_count = 0
        confidences = []
    
    # 判定状态
    status_text, status_code = _determine_status(pcb_count)
    
    # 保存结果图片
    if output_dir:
        output_dir = Path(output_dir)
    else:
        output_dir = DEFAULT_OUTPUT_DIR / "single"
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    img = cv2.imread(str(image_path))
    img = _draw_results(img, results, pcb_count)
    
    output_path = output_dir / f"result_{image_path.name}"
    cv2.imwrite(str(output_path), img)
    print(f"结果已保存到: {output_path}")
    
    # 打印结果
    print(f"\n{'='*50}")
    print(f"图片: {image_path.name}")
    print(f"检测结果:")
    print(f"  - PCB数量: {pcb_count} 个")
    if confidences:
        print(f"  - 置信度: {[f'{c:.2f}' for c in confidences]}")
    print(f"判定状态: {status_text}")
    print(f"{'='*50}\n")
    
    return {
        "image": str(image_path),
        "pcb_count": pcb_count,
        "confidences": confidences,
        "status": status_text,
        "status_code": status_code,
    }


def predict_batch(image_dir: str, output_dir: str = None, conf_threshold: float = DEFAULT_CONF):
    """
    批量检测图片并计数
    """
    image_dir = Path(image_dir)
    if not image_dir.exists():
        raise FileNotFoundError(f"未找到图片目录: {image_dir}")
    
    # 查找模型
    try:
        weight_path = find_best_weight()
    except FileNotFoundError as e:
        raise FileNotFoundError(f"{e}\n请先训练模型或检查 {MODELS_DIR} 目录")
    
    # 设置输出目录
    if output_dir is None:
        output_dir = DEFAULT_OUTPUT_DIR / "batch" / image_dir.name
    else:
        output_dir = Path(output_dir)
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 加载模型
    model = YOLO(str(weight_path))
    print(f"已加载模型: {weight_path}")
    print(f"置信度阈值: {conf_threshold}")
    print(f"输出目录: {output_dir}")  # 添加这行确认输出目录
    
    # 获取所有图片
    image_files = []
    for ext in ["*.png", "*.jpg", "*.jpeg"]:
        image_files.extend(image_dir.glob(ext))
    image_files.sort()
    
    print(f"找到 {len(image_files)} 张图片\n")
    
    # 批量处理
    all_results = []
    stats = {
        "total": len(image_files),
        "empty": 0,
        "total_pcbs": 0,
    }
    
    for img_path in image_files:
        # 预测 - 关键修改：设置 save=False 避免自动保存
        results = model.predict(
            source=str(img_path),
            save=False,           # 确保不自动保存
            project=None,         # 不设置project
            name=None,            # 不设置name
            exist_ok=False,
            imgsz=IMGSZ,
            conf=conf_threshold,
        )[0]
        
        # 统计
        pcb_count = 0
        confidences = []
        if results.boxes is not None:
            pcb_count = len(results.boxes)
            confidences = [float(conf) for conf in results.boxes.conf]
        
        stats["total_pcbs"] += pcb_count
        if pcb_count == 0:
            stats["empty"] += 1
        
        # 判定状态
        status_text, status_code = _determine_status(pcb_count)
        
        # 保存结果图片到指定目录
        img = cv2.imread(str(img_path))
        img = _draw_results(img, results, pcb_count)
        output_path = output_dir / img_path.name
        cv2.imwrite(str(output_path), img)
        
        # 保存检测结果JSON
        result_json = output_dir / f"{img_path.stem}.json"
        with open(result_json, "w", encoding="utf-8") as f:
            json.dump({
                "image": img_path.name,
                "pcb_count": pcb_count,
                "confidences": confidences,
                "status": status_text,
                "status_code": status_code,
            }, f, indent=2)
        
        # 记录结果
        result = {
            "image": img_path.name,
            "pcb_count": pcb_count,
            "confidences": confidences,
            "status": status_text,
            "status_code": status_code,
        }
        all_results.append(result)
        
        # 实时打印
        status_symbol = "🟢" if pcb_count > 0 else "⚪"
        conf_str = f" (置信度: {[f'{c:.2f}' for c in confidences]})" if confidences else ""
        print(f"{img_path.name}: {status_symbol} {status_text}{conf_str}")
    
    # 保存汇总结果
    summary_file = output_dir / "summary.json"
    with open(summary_file, "w", encoding="utf-8") as f:
        json.dump({
            "statistics": {
                "total_images": stats["total"],
                "empty_boxes": stats["empty"],
                "non_empty": stats["total"] - stats["empty"],
                "total_pcbs": stats["total_pcbs"],
                "average_pcbs": stats["total_pcbs"] / (stats["total"] - stats["empty"]) if stats["total"] - stats["empty"] > 0 else 0
            },
            "details": all_results
        }, f, indent=2, ensure_ascii=False)
    
    # 打印统计
    print(f"\n{'='*50}")
    print("📊 批量检测统计")
    print(f"{'='*50}")
    print(f"总图片数: {stats['total']} 张")
    print(f"空料箱: {stats['empty']} 个")
    print(f"有PCB: {stats['total'] - stats['empty']} 个")
    print(f"总PCB数量: {stats['total_pcbs']} 个")
    if stats['total'] - stats['empty'] > 0:
        print(f"平均每箱PCB数: {stats['total_pcbs']/(stats['total'] - stats['empty']):.2f}")
    print(f"\n结果已保存到: {output_dir}")
    print(f"{'='*50}")
    
    return all_results


def predict_stream(source: str | int = 0, conf_threshold: float = DEFAULT_CONF):
    """
    摄像头实时检测（智能版）
    自动检测并适应不同摄像头
    图片上显示英文，终端输出保持中文
    """
    # 查找模型
    try:
        weight_path = find_best_weight()
    except FileNotFoundError as e:
        raise FileNotFoundError(f"{e}\n请先训练模型或检查 {MODELS_DIR} 目录")
    
    print(f"\n{'='*60}")
    print("🚀 启动摄像头实时检测（智能适配版）")
    print(f"{'='*60}")
    print(f"使用模型: {weight_path}")
    print(f"置信度阈值: {conf_threshold}")
    
    # 加载模型
    model = YOLO(str(weight_path))
    
    # 打开摄像头
    if isinstance(source, str) and source.isdigit():
        source = int(source)
    
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise ValueError(f"无法打开摄像头: {source}")
    
    # ========== 摄像头信息自动检测（仿detect.py） ==========
    print("\n📷 正在检测摄像头信息...")
    
    # 获取摄像头原始分辨率
    orig_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    orig_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    orig_fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"  原始分辨率: {orig_width} x {orig_height}")
    print(f"  原始帧率: {orig_fps:.2f} fps")
    
    # 检测摄像头支持的其他分辨率
    supported_resolutions = []
    test_resolutions = [
        (3840, 2160),  # 4K
        (2560, 1440),  # 2K
        (1920, 1080),  # 1080p
        (1280, 720),   # 720p
        (800, 450),    # 你的原生分辨率
        (640, 480),    # VGA
        (IMGSZ, int(IMGSZ * 9/16)),  # 模型输入尺寸的16:9
        (IMGSZ, IMGSZ),               # 正方形
    ]
    
    for width, height in test_resolutions:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        test_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        test_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        if test_width == width and test_height == height:
            supported_resolutions.append((width, height))
            print(f"  ✅ 支持: {width} x {height}")
        else:
            print(f"  ❌ 不支持: {width} x {height}")
    
    # 智能选择最佳分辨率
    target_width = IMGSZ
    target_height = int(IMGSZ * 9/16)  # 16:9 比例
    
    # 优先选择与目标最接近的支持分辨率
    best_res = (orig_width, orig_height)
    min_diff = float('inf')
    
    for width, height in supported_resolutions:
        # 计算与目标分辨率的差距（考虑比例）
        ratio_diff = abs(width/height - target_width/target_height)
        size_diff = abs(width - target_width) + abs(height - target_height)
        total_diff = size_diff + ratio_diff * 1000  # 比例差异加权
        
        if total_diff < min_diff:
            min_diff = total_diff
            best_res = (width, height)
    
    # 设置最佳分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, best_res[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, best_res[1])
    
    # 尝试设置较高帧率
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    # 最终确认
    final_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    final_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    final_fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"\n📊 最终使用配置:")
    print(f"  - 分辨率: {final_width} x {final_height}")
    print(f"  - 画面比例: {final_width/final_height:.2f}")
    print(f"  - 帧率: {final_fps:.2f} fps")
    print(f"  - 模型输入尺寸: {IMGSZ} x {IMGSZ}")
    print(f"{'='*60}\n")
    
    print("按键说明:")
    print("  - 'q': 退出")
    print("  - 's': 保存当前帧")
    print("  - 'i': 显示摄像头信息")
    print("  - 'r': 重置分辨率")
    print("  - 'c': 清零累计计数")
    print(f"{'='*60}\n")
    
    # 创建保存目录 - 指定在 bin_pcb_detection 下
    save_dir = BIN_ROOT / "results" / "inference" / "stream_captures"
    save_dir.mkdir(parents=True, exist_ok=True)
    print(f"📁 截图保存目录: {save_dir}")
    print(f"{'='*60}\n")
    
    # 创建可调整大小的窗口
    window_name = "Bin PCB Detection - Press 'q' to quit, 's' to save"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    # 统计变量
    frame_count = 0
    total_pcbs_detected = 0
    fps_timer = cv2.getTickCount()
    fps_display = 0
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法获取摄像头画面")
                break
            
            # 计算实时帧率
            frame_count += 1
            if frame_count % 30 == 0:
                current_time = cv2.getTickCount()
                time_diff = (current_time - fps_timer) / cv2.getTickFrequency()
                fps_display = 30 / time_diff if time_diff > 0 else 0
                fps_timer = current_time
            
            # 运行检测
            results = model(frame, imgsz=IMGSZ, conf=conf_threshold)[0]
            
            # 统计PCB数量
            pcb_count = 0
            if results.boxes is not None:
                pcb_count = len(results.boxes)
                total_pcbs_detected += pcb_count
            
            # 判定状态（用于终端，保持中文）
            status_text, status_code = _determine_status(pcb_count)
            
            # 在画面上绘制结果
            annotated_frame = results.plot()
            
            # 英文状态文字（用于图像显示）
            if pcb_count == 0:
                status_text_en = "Empty Box"
                status_color = (0, 0, 255)  # 红色
            else:
                status_text_en = f"Has {pcb_count} PCB"
                status_color = (0, 255, 0)  # 绿色
            
            # 添加状态和信息文字（全部用英文）
            info_lines = [
                f"Status: {status_text_en}",
                f"Current: {pcb_count}  Total: {total_pcbs_detected}",
                f"Resolution: {final_width}x{final_height}",
                f"FPS: {fps_display:.1f}",
                f"Model Size: {IMGSZ}x{IMGSZ}"
            ]
            
            for i, line in enumerate(info_lines):
                y_pos = 30 + i * 30
                color = status_color if i < 2 else (255, 255, 255)
                cv2.putText(annotated_frame, line, (10, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # 显示画面
            cv2.imshow(window_name, annotated_frame)
            
            # 处理按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("👋 用户退出")
                break
            elif key == ord('s'):
                # 保存当前帧
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                save_path = save_dir / f"capture_{timestamp}_pcb{pcb_count}.jpg"
                cv2.imwrite(str(save_path), annotated_frame)
                print(f"✅ 已保存: {save_path}")
            elif key == ord('i'):
                # 显示详细摄像头信息
                print("\n📷 摄像头详细信息:")
                print(f"  - 当前分辨率: {final_width} x {final_height}")
                print(f"  - 当前帧率: {final_fps:.2f} fps")
                print(f"  - 亮度: {cap.get(cv2.CAP_PROP_BRIGHTNESS)}")
                print(f"  - 对比度: {cap.get(cv2.CAP_PROP_CONTRAST)}")
                print(f"  - 饱和度: {cap.get(cv2.CAP_PROP_SATURATION)}")
                print(f"  - 增益: {cap.get(cv2.CAP_PROP_GAIN)}")
            elif key == ord('r'):
                # 重置分辨率到原始值
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, orig_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, orig_height)
                final_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                final_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(f"分辨率已重置为: {final_width} x {final_height}")
            elif key == ord('c'):
                # 清零累计计数
                total_pcbs_detected = 0
                print("🔄 累计计数已清零")
    
    except KeyboardInterrupt:
        print("\n👋 用户中断程序")
    except Exception as e:
        print(f"❌ 错误: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("摄像头检测已停止")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="物料箱PCB检测计数（仿detect.py风格）")
    parser.add_argument("--mode", type=str, default="single",
                        choices=["single", "batch", "stream"],
                        help="运行模式: single(单张图片), batch(批量图片), stream(摄像头)")
    parser.add_argument("--image", type=str, default=None,
                        help="单张图片路径 (同 --mode single)")
    parser.add_argument("--dir", type=str, default=None,
                        help="图片目录路径 (同 --mode batch)")
    parser.add_argument("--stream", type=str, nargs='?', const='0',
                        help="摄像头实时检测 (可指定摄像头ID或视频文件)")
    parser.add_argument("--source", type=str, default=None,
                        help="图片路径、目录或摄像头ID (兼容detect.py)")
    parser.add_argument("--output", type=str, default=None,
                        help="输出目录 (默认: results/inference)")
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF,
                        help=f"置信度阈值 (默认: {DEFAULT_CONF})")
    
    args = parser.parse_args()
    
    # 兼容detect.py的参数风格
    if args.stream is not None:
        # stream模式
        source = args.stream
        predict_stream(source, args.conf)
    elif args.image is not None:
        # 单张图片模式
        predict_single_image(args.image, args.output, args.conf)
    elif args.dir is not None:
        # 批量图片模式
        predict_batch(args.dir, args.output, args.conf)
    elif args.source is not None:
        # 根据source智能判断
        source_path = Path(args.source)
        if source_path.is_dir():
            predict_batch(args.source, args.output, args.conf)
        elif source_path.is_file():
            predict_single_image(args.source, args.output, args.conf)
        else:
            # 可能是摄像头ID
            try:
                int(args.source)
                predict_stream(args.source, args.conf)
            except ValueError:
                print(f"错误: 无法识别的source: {args.source}")
    else:
        # 没有参数，显示帮助信息
        print("\n" + "="*60)
        print("请指定以下参数之一：")
        print("="*60)
        print("  --stream        摄像头实时检测")
        print("  --image FILE    单张图片")
        print("  --dir DIR       批量处理目录")
        print("  --source PATH   图片路径/目录/摄像头ID")
        print("\n示例:")
        print(f"  python3 {Path(__file__).name} --stream")
        print(f"  python3 {Path(__file__).name} --image test.jpg")
        print(f"  python3 {Path(__file__).name} --dir ./images")
        print(f"  python3 {Path(__file__).name} --source ./images")
        print("="*60 + "\n")