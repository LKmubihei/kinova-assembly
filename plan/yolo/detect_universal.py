#!/usr/bin/env python3
"""
通用实时检测脚本 - 支持任何相机和任何项目
"""

from pathlib import Path
from ultralytics import YOLO
import cv2
import argparse
import numpy as np
import time

# ==================== 配置区域 ====================
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR

# 项目映射
PROJECTS = {
    "power_panel": {
        "name": "电源板检测",
        "model_dir": PROJECT_ROOT / "power_panel_detection" / "models",
        "model_name": "best.pt",
        "classes": {0: "power_panel", 1: "peg"},
        "colors": {0: (0, 255, 0), 1: (255, 0, 0)},
        "needs_two_args": True,  # 需要两个参数
        "status_func": None  # 将在后面定义
    },
    "bin_pcb": {
        "name": "物料箱PCB检测",
        "model_dir": PROJECT_ROOT / "bin_pcb_detection" / "models",
        "model_name": "bin_model.pt",
        "classes": {0: "pcb"},
        "colors": {0: (0, 255, 0)},
        "needs_two_args": False,  # 只需要一个参数
        "status_func": None  # 将在后面定义
    }
}

# 相机类型配置
CAMERA_TYPES = {
    "usb": {
        "name": "USB摄像头",
        "init_func": lambda source: cv2.VideoCapture(int(source) if source.isdigit() else source),
        "read_func": lambda cap: cap.read()
    },
    "realsense": {
        "name": "Intel RealSense",
        "init_func": None,  # 将在后面定义
        "read_func": None   # 将在后面定义
    }
}

DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "results" / "inference"
DEFAULT_CONF = 0.25
IMGSZ = 800
# =================================================


# ==================== 状态判定函数（返回英文）====================
def _determine_power_status(power_count: int, peg_count: int) -> tuple[str, tuple]:
    """
    电源板状态判定
    返回英文状态用于图像显示
    """
    if power_count == 0:
        return "No Power Panel", (0, 0, 255)  # 红色：没有电源板
    elif power_count > 0 and peg_count == 4:
        return "Correct", (0, 255, 0)  # 绿色：正确放置
    elif power_count > 0 and peg_count != 4:
        return "Incorrect", (0, 165, 255)  # 橙色：未正确放置
    else:
        return "Unknown", (255, 255, 255)  # 白色：未知状态


def _determine_bin_status(pcb_count: int, peg_count: int = 0) -> tuple[str, tuple]:
    """
    物料箱PCB状态判定
    接收两个参数但只使用第一个，兼容通用调用
    返回英文状态用于图像显示
    """
    if pcb_count == 0:
        return "Empty Box", (0, 0, 255)  # 红色：空料箱
    else:
        return f"Has {pcb_count} PCB", (0, 255, 0)  # 绿色：有PCB


# 更新项目映射中的状态函数
PROJECTS["power_panel"]["status_func"] = _determine_power_status
PROJECTS["bin_pcb"]["status_func"] = _determine_bin_status


# ==================== RealSense 相关函数 ====================
def _init_realsense(source):
    """初始化 RealSense 相机"""
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("❌ 未安装 pyrealsense2，请运行: pip install pyrealsense2")
        return None
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    if source and source != "0":
        config.enable_device(source)
    
    # 尝试配置
    try:
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        print("✅ 配置: 1280x720 @ 30fps")
    except:
        try:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            print("✅ 配置: 640x480 @ 30fps")
        except:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
            print("✅ 配置: 640x480 @ 60fps")
    
    try:
        pipeline.start(config)
        print("✅ RealSense 相机启动成功")
        return pipeline
    except Exception as e:
        print(f"❌ RealSense 相机启动失败: {e}")
        return None


def _read_realsense(cap):
    """读取 RealSense 帧"""
    try:
        import pyrealsense2 as rs
        frames = cap.wait_for_frames(timeout_ms=5000)
        color_frame = frames.get_color_frame()
        if color_frame:
            return True, np.asanyarray(color_frame.get_data())
        return False, None
    except Exception as e:
        print(f"读取 RealSense 帧失败: {e}")
        return False, None


# 更新相机配置
CAMERA_TYPES["realsense"]["init_func"] = _init_realsense
CAMERA_TYPES["realsense"]["read_func"] = _read_realsense


# ==================== 通用函数 ====================
def find_model(project: str) -> Path:
    """查找项目模型"""
    project_config = PROJECTS.get(project)
    if not project_config:
        return None
    
    model_path = project_config["model_dir"] / project_config["model_name"]
    if model_path.exists():
        return model_path
    
    # 尝试查找其他模型
    other_models = list(project_config["model_dir"].glob("*.pt"))
    if other_models:
        print(f"⚠️ 未找到 {project_config['model_name']}，使用: {other_models[0].name}")
        return other_models[0]
    
    return None


def detect_stream(project: str, camera_type: str = "usb", 
                  source: str = "0", conf_threshold: float = DEFAULT_CONF):
    """
    通用实时检测函数
    """
    # 获取项目配置
    project_config = PROJECTS.get(project)
    if not project_config:
        print(f"❌ 未知项目: {project}")
        print(f"可用项目: {list(PROJECTS.keys())}")
        return
    
    # 查找模型
    model_path = find_model(project)
    if not model_path:
        print(f"❌ 未找到 {project} 的模型")
        print(f"请检查: {project_config['model_dir']}")
        return
    
    # 加载模型
    print(f"\n{'='*60}")
    print(f"🚀 启动 {project_config['name']} 实时检测")
    print(f"{'='*60}")
    print(f"项目: {project_config['name']}")
    print(f"相机类型: {CAMERA_TYPES[camera_type]['name']}")
    print(f"使用模型: {model_path}")
    print(f"置信度阈值: {conf_threshold}")
    
    try:
        model = YOLO(str(model_path))
        print("✅ YOLO 模型加载成功")
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return
    
    # 初始化相机
    camera_config = CAMERA_TYPES.get(camera_type)
    if not camera_config:
        print(f"❌ 未知相机类型: {camera_type}")
        return
    
    try:
        cap = camera_config["init_func"](source)
        if cap is None:
            print("❌ 相机初始化失败")
            return
        
        # 获取相机信息
        if camera_type == "usb":
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = cap.get(cv2.CAP_PROP_FPS)
        else:
            width, height, fps = 1280, 720, 30  # RealSense 默认
    except Exception as e:
        print(f"❌ 相机初始化失败: {e}")
        return
    
    print(f"\n📷 相机信息:")
    print(f"  分辨率: {width} x {height}")
    if fps > 0:
        print(f"  帧率: {fps:.2f} fps")
    
    print("\n按键说明:")
    print("  - 'q': 退出")
    print("  - 's': 保存当前帧")
    print("  - 'c': 切换置信度阈值")
    print(f"{'='*60}\n")
    
    # 创建保存目录
    save_dir = DEFAULT_OUTPUT_DIR / f"{project}_{camera_type}_captures"
    save_dir.mkdir(parents=True, exist_ok=True)
    
    # 统计变量
    frame_count = 0
    total_detected = 0
    fps_timer = cv2.getTickCount()
    fps_display = 0
    current_conf = conf_threshold
    
    window_name = f"{project_config['name']} - {CAMERA_TYPES[camera_type]['name']}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    try:
        while True:
            # 读取帧
            if camera_type == "usb":
                ret, frame = cap.read()
            else:
                ret, frame = camera_config["read_func"](cap)
            
            if not ret or frame is None:
                print("⚠️ 无法获取画面，等待重试...")
                time.sleep(0.1)
                continue
            
            # 计算帧率
            frame_count += 1
            if frame_count % 10 == 0:
                current_time = cv2.getTickCount()
                time_diff = (current_time - fps_timer) / cv2.getTickFrequency()
                fps_display = 10 / time_diff if time_diff > 0 else 0
                fps_timer = current_time
            
            # 运行检测
            results = model(frame, imgsz=IMGSZ, conf=current_conf)[0]
            
            # 统计检测结果
            counts = {}
            if results.boxes is not None:
                for cls in results.boxes.cls:
                    cls_id = int(cls)
                    counts[cls_id] = counts.get(cls_id, 0) + 1
                total_detected += len(results.boxes)
            
            # 判定状态（根据项目类型决定参数数量）
            if project_config["needs_two_args"]:
                status_text, status_color = project_config["status_func"](
                    counts.get(0, 0), counts.get(1, 0))
            else:
                status_text, status_color = project_config["status_func"](
                    counts.get(0, 0))
            
            # 绘制结果
            annotated_frame = results.plot()
            
            # 构建检测结果字符串（英文）
            det_result = " ".join([f"{project_config['classes'][k]}:{v}" for k, v in counts.items()])
            if not det_result:
                det_result = "No detections"
            
            # 添加信息（全部用英文）
            info_lines = [
                f"Status: {status_text}",
                f"Detected: {det_result}",
                f"Resolution: {width}x{height}",
                f"FPS: {fps_display:.1f}",
                f"Total: {total_detected}",
                f"Conf: {current_conf:.2f}"
            ]
            
            for i, line in enumerate(info_lines):
                y_pos = 30 + i * 30
                color = status_color if i < 2 else (255, 255, 255)
                cv2.putText(annotated_frame, line, (10, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            cv2.imshow(window_name, annotated_frame)
            
            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("👋 用户退出")
                break
            elif key == ord('s'):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                save_path = save_dir / f"{timestamp}.jpg"
                cv2.imwrite(str(save_path), annotated_frame)
                print(f"✅ 已保存: {save_path}")
            elif key == ord('c'):
                # 循环切换置信度阈值
                if current_conf < 0.2:
                    current_conf = 0.35
                elif current_conf < 0.3:
                    current_conf = 0.15
                else:
                    current_conf = 0.25
                print(f"📊 置信度阈值调整为: {current_conf:.2f}")
    
    except KeyboardInterrupt:
        print("\n👋 用户中断程序")
    except Exception as e:
        print(f"❌ 运行时错误: {e}")
    finally:
        if camera_type == "usb":
            cap.release()
        cv2.destroyAllWindows()
        print("检测已停止")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="通用实时检测脚本")
    parser.add_argument("--project", type=str, default="power_panel",
                        choices=list(PROJECTS.keys()),
                        help=f"项目类型: {list(PROJECTS.keys())}")
    parser.add_argument("--camera", type=str, default="usb",
                        choices=list(CAMERA_TYPES.keys()),
                        help=f"相机类型: {list(CAMERA_TYPES.keys())}")
    parser.add_argument("--source", type=str, default="0",
                        help="相机源 (USB: 设备号, RealSense: 序列号)")
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF,
                        help=f"置信度阈值 (默认: {DEFAULT_CONF})")
    
    args = parser.parse_args()
    
    try:
        detect_stream(args.project, args.camera, args.source, args.conf)
    except KeyboardInterrupt:
        print("\n👋 用户中断程序")
    except Exception as e:
        print(f"❌ 错误: {e}")