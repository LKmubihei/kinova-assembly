#!/usr/bin/env python3
"""
推理脚本 - 电源板装配状态判定
"""

from pathlib import Path
from ultralytics import YOLO
import json
import cv2
import argparse
import time

# ==================== 配置区域 ====================
SCRIPT_DIR = Path(__file__).resolve().parent
# 项目根目录 (scripts的上一级)
PROJECT_ROOT = SCRIPT_DIR.parent
# 模型目录
MODELS_DIR = PROJECT_ROOT / "models"
# 默认输出目录
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "results" / "inference"

# 模型路径（相对路径）
BEST_WEIGHT = MODELS_DIR / "best.pt"

# 图片尺寸（必须与训练时一致）
IMGSZ = 800
# =================================================


def _determine_status(power_count: int, peg_count: int) -> tuple[str, str]:
    """
    根据检测结果判定装配状态
    返回: (状态描述, 状态码) - 用于终端输出，保持中文
    """
    if power_count == 0:
        return "没有电源板", "no_panel"
    elif power_count > 0 and peg_count == 4:
        return "正确放置", "correct"
    elif power_count > 0 and peg_count != 4:
        return "未正确放置", "incorrect"
    else:
        return "未知状态", "unknown"


def _get_status_en(power_count: int, peg_count: int) -> tuple[str, tuple]:
    """
    获取英文状态文字和颜色 - 用于图像显示
    """
    if power_count == 0:
        return "No Power Panel", (0, 0, 255)  # 红色
    elif power_count > 0 and peg_count == 4:
        return "Correct", (0, 255, 0)  # 绿色
    elif power_count > 0 and peg_count != 4:
        return "Incorrect", (0, 165, 255)  # 橙色
    else:
        return "Unknown", (255, 255, 255)  # 白色


def _draw_results(image, results, power_count: int, peg_count: int):
    """
    在图片上绘制检测结果和状态（全部用英文）
    """
    # 绘制检测框
    if results.boxes is not None:
        for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
            x1, y1, x2, y2 = map(int, box.tolist())
            label = "power_panel" if int(cls) == 0 else "peg"
            color = (0, 255, 0) if label == "power_panel" else (255, 0, 0)
            
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(image, f"{label} {conf:.2f}", (x1, y1-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    # 获取英文状态文字和颜色
    status_text_en, status_color = _get_status_en(power_count, peg_count)
    
    # 添加状态文字（英文）
    cv2.putText(image, f"Status: {status_text_en}", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
    cv2.putText(image, f"Power: {power_count}  Peg: {peg_count}", (10, 60),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
    
    return image


def predict_single_image(image_path: str, output_dir: str = None):
    """
    单张图片预测并判定状态
    """
    image_path = Path(image_path)
    
    if not BEST_WEIGHT.exists():
        raise FileNotFoundError(f"未找到训练权重: {BEST_WEIGHT}\n"
                               f"请先训练模型或检查 {MODELS_DIR} 目录")
    
    if not image_path.exists():
        raise FileNotFoundError(f"未找到图片: {image_path}")
    
    # 加载模型
    model = YOLO(str(BEST_WEIGHT))
    
    # 预测
    results = model.predict(
        source=str(image_path),
        save=False,
        imgsz=IMGSZ,
        conf=0.25,
    )[0]
    
    # 统计各类别数量
    power_count = 0
    peg_count = 0
    
    if results.boxes is not None:
        for cls in results.boxes.cls:
            if int(cls) == 0:
                power_count += 1
            elif int(cls) == 1:
                peg_count += 1
    
    # 判定状态（用于终端，中文）
    status_text, status_code = _determine_status(power_count, peg_count)
    
    # 保存结果图片
    if output_dir:
        output_dir = Path(output_dir)
    else:
        output_dir = DEFAULT_OUTPUT_DIR / "single"
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    img = cv2.imread(str(image_path))
    img = _draw_results(img, results, power_count, peg_count)
    
    output_path = output_dir / f"result_{image_path.name}"
    cv2.imwrite(str(output_path), img)
    print(f"结果已保存到: {output_path}")
    
    # 打印结果（终端用中文）
    print(f"\n{'='*50}")
    print(f"图片: {image_path.name}")
    print(f"检测结果:")
    print(f"  - Power Panel: {power_count} 个")
    print(f"  - Peg: {peg_count} 个")
    print(f"判定状态: {status_text}")
    print(f"{'='*50}\n")
    
    return {
        "image": str(image_path),
        "power_panel": power_count,
        "peg": peg_count,
        "status": status_text,
        "status_code": status_code,
    }


def predict_batch(image_dir: str, output_dir: str = None):
    """
    批量预测图片
    """
    image_dir = Path(image_dir)
    if not image_dir.exists():
        raise FileNotFoundError(f"未找到图片目录: {image_dir}")
    
    if output_dir is None:
        output_dir = DEFAULT_OUTPUT_DIR / "batch" / image_dir.name
    else:
        output_dir = Path(output_dir)
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 加载模型
    if not BEST_WEIGHT.exists():
        raise FileNotFoundError(f"未找到训练权重: {BEST_WEIGHT}")
    
    model = YOLO(str(BEST_WEIGHT))
    
    # 获取所有图片
    image_files = []
    for ext in ["*.png", "*.jpg", "*.jpeg"]:
        image_files.extend(image_dir.glob(ext))
    image_files.sort()
    
    print(f"找到 {len(image_files)} 张图片")
    
    # 批量处理
    all_results = []
    stats = {
        "total": len(image_files),
        "no_panel": 0,
        "correct": 0,
        "incorrect": 0,
    }
    
    for img_path in image_files:
        # 预测
        results = model.predict(
            source=str(img_path),
            save=False,
            imgsz=IMGSZ,
            conf=0.25,
        )[0]
        
        # 统计
        power_count = 0
        peg_count = 0
        if results.boxes is not None:
            for cls in results.boxes.cls:
                if int(cls) == 0:
                    power_count += 1
                elif int(cls) == 1:
                    peg_count += 1
        
        # 判定（用于终端，中文）
        status_text, status_code = _determine_status(power_count, peg_count)
        stats[status_code] += 1
        
        # 保存结果图片（用英文绘制）
        img = cv2.imread(str(img_path))
        img = _draw_results(img, results, power_count, peg_count)
        output_path = output_dir / img_path.name
        cv2.imwrite(str(output_path), img)
        
        # 记录结果
        result = {
            "image": img_path.name,
            "power_panel": power_count,
            "peg": peg_count,
            "status": status_text,
            "status_code": status_code,
        }
        all_results.append(result)
        
        print(f"{img_path.name}: {status_text} (Power: {power_count}, Peg: {peg_count})")
    
    # 保存详细结果
    results_json = output_dir / "results.json"
    with open(results_json, "w", encoding="utf-8") as f:
        json.dump({
            "statistics": stats,
            "details": all_results
        }, f, indent=2, ensure_ascii=False)
    
    # 打印统计（终端用中文）
    print(f"\n{'='*50}")
    print("批量预测统计")
    print(f"{'='*50}")
    print(f"总图片数: {stats['total']}")
    print(f"✅ 正确放置: {stats['correct']}")
    print(f"⚠️ 未正确放置: {stats['incorrect']}")
    print(f"❌ 没有电源板: {stats['no_panel']}")
    print(f"\n结果已保存到: {output_dir}")
    print(f"{'='*50}")
    
    return all_results


def predict_stream(source: str | int = 0):
    """
    摄像头实时检测（智能版）- 自动检测并适应不同摄像头
    图片上显示英文，终端输出保持中文
    """
    if not BEST_WEIGHT.exists():
        raise FileNotFoundError(f"未找到训练权重: {BEST_WEIGHT}")

    # 加载模型
    model = YOLO(str(BEST_WEIGHT))
    
    print(f"\n{'='*60}")
    print("🚀 启动摄像头实时检测（智能适配版）")
    print(f"{'='*60}")
    print(f"使用模型: {BEST_WEIGHT}")
    
    # 打开摄像头
    if isinstance(source, str) and source.isdigit():
        source = int(source)
    
    cap = cv2.VideoCapture(source)
    
    if not cap.isOpened():
        raise ValueError(f"无法打开摄像头: {source}")
    
    # ========== 摄像头信息自动检测 ==========
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
    print(f"{'='*60}\n")
    
    # 创建保存目录
    save_dir = DEFAULT_OUTPUT_DIR / "stream_captures"
    save_dir.mkdir(parents=True, exist_ok=True)
    
    # 创建可调整大小的窗口
    window_name = "Power Panel Detection - Press 'q' to quit, 's' to save"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    frame_count = 0
    total_detected = 0
    fps_timer = cv2.getTickCount()
    fps_display = 0
    
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
        results = model(frame, imgsz=IMGSZ, conf=0.25)[0]
        
        # 统计各类别数量
        power_count = 0
        peg_count = 0
        if results.boxes is not None:
            for cls in results.boxes.cls:
                if int(cls) == 0:
                    power_count += 1
                elif int(cls) == 1:
                    peg_count += 1
            total_detected += len(results.boxes)
        
        # 判定状态（用于终端，中文）
        status_text, status_code = _determine_status(power_count, peg_count)
        
        # 在画面上绘制结果（用英文）
        annotated_frame = results.plot()
        
        # 获取英文状态文字和颜色
        status_text_en, status_color = _get_status_en(power_count, peg_count)
        
        # 添加状态和信息文字（全部用英文）
        info_lines = [
            f"Status: {status_text_en}",
            f"Power: {power_count}  Peg: {peg_count}",
            f"Resolution: {final_width}x{final_height}",
            f"FPS: {fps_display:.1f}",
            f"Total: {total_detected}"
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
            save_path = save_dir / f"capture_{timestamp}_power{power_count}_peg{peg_count}.jpg"
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
            print(f"  - 色调: {cap.get(cv2.CAP_PROP_HUE)}")
            print(f"  - 增益: {cap.get(cv2.CAP_PROP_GAIN)}")
            print(f"  - 曝光: {cap.get(cv2.CAP_PROP_EXPOSURE)}")
        elif key == ord('r'):
            # 重置分辨率到原始值
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, orig_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, orig_height)
            final_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            final_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"分辨率已重置为: {final_width} x {final_height}")
    
    cap.release()
    cv2.destroyAllWindows()
    print("摄像头检测已停止")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="电源板装配状态检测")
    parser.add_argument("--image", type=str, help="单张图片路径")
    parser.add_argument("--dir", type=str, help="图片目录路径")
    parser.add_argument("--stream", type=str, nargs='?', const='0', 
                       help="摄像头实时检测（可指定摄像头ID或视频文件）")
    parser.add_argument("--output", type=str, help="输出目录（默认: results/inference）")
    
    args = parser.parse_args()
    
    try:
        if args.stream is not None:
            if args.stream == '0':
                predict_stream(0)
            else:
                predict_stream(args.stream)
        elif args.image:
            predict_single_image(args.image, args.output)
        elif args.dir:
            predict_batch(args.dir, args.output)
        else:
            print("请指定以下参数之一：")
            print("  --stream       摄像头实时检测")
            print("  --image FILE   单张图片")
            print("  --dir DIR      批量处理目录")
            print("\n示例:")
            print(f"  python3 {Path(__file__).name} --stream")
            print(f"  python3 {Path(__file__).name} --image test.jpg")
            print(f"  python3 {Path(__file__).name} --dir ./images")
    except KeyboardInterrupt:
        print("\n👋 用户中断程序")
    except Exception as e:
        print(f"❌ 错误: {e}")