#!/usr/bin/env python3
"""
训练脚本 - 电源板装配检测
"""

from pathlib import Path
from ultralytics import YOLO
import torch
import shutil
import os

# ==================== 配置区域 ====================
# 获取当前脚本所在目录的绝对路径
SCRIPT_DIR = Path(__file__).resolve().parent
# 项目根目录 (scripts的上一级)
PROJECT_ROOT = SCRIPT_DIR.parent
# 数据集目录
DATA_DIR = PROJECT_ROOT / "data"
# 结果保存目录
RESULTS_DIR = PROJECT_ROOT / "results"
# 模型保存目录
MODELS_DIR = PROJECT_ROOT / "models"

# 数据集配置文件
DATA_YAML = DATA_DIR / "data.yaml"

# 预训练模型名称（会自动从网上下载）
MODEL_NAME = "yolo11n.pt"

# 图片尺寸设置（根据你的原始图片调整）
IMGSZ = 800  # 你的原始图片是800x450，YOLO会缩放到800x800正方形

# RTX 5060 优化配置
os.environ["CUDA_VISIBLE_DEVICES"] = "0"  # 指定使用GPU 0
os.environ["TORCH_CUDNN_V8_API_ENABLED"] = "1"  # 启用cuDNN v8 API
# =================================================


def check_gpu():
    """检查GPU状态并打印信息"""
    if not torch.cuda.is_available():
        print("⚠️ CUDA不可用，将使用CPU训练（速度会很慢）")
        return False
    
    # 获取GPU信息
    gpu_name = torch.cuda.get_device_name(0)
    gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1e9  # GB
    capability = torch.cuda.get_device_capability(0)
    
    print(f"\n{'='*50}")
    print("🎮 GPU 信息")
    print(f"{'='*50}")
    print(f"GPU型号: {gpu_name}")
    print(f"显存大小: {gpu_memory:.1f} GB")
    print(f"计算能力: {capability[0]}.{capability[1]}")
    print(f"PyTorch版本: {torch.__version__}")
    print(f"CUDA版本: {torch.version.cuda}")
    print(f"训练图片尺寸: {IMGSZ}×{IMGSZ}")
    print(f"项目根目录: {PROJECT_ROOT}")
    
    # 检查是否为Blackwell架构 (sm_120)
    arch_list = torch.cuda.get_arch_list()
    print(f"支持的架构: {arch_list}")
    
    if any('sm_120' in arch for arch in arch_list):
        print("✅ 当前PyTorch支持sm_120，可以充分发挥RTX 5060性能！")
    else:
        print("⚠️ 当前PyTorch不支持sm_120，性能会受限")
        print("   建议安装支持sm_120的PyTorch版本以获得最佳性能")
    
    return True


def ensure_dirs():
    """确保所有必要的目录存在"""
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    print(f"📁 结果保存目录: {RESULTS_DIR}")
    print(f"📁 模型保存目录: {MODELS_DIR}")


def train_detect():
    """训练 YOLOv11 目标检测模型（针对RTX 5060优化）"""
    
    # 确保目录存在
    ensure_dirs()
    
    # 检查配置文件
    if not DATA_YAML.exists():
        raise FileNotFoundError(f"未找到数据集配置文件: {DATA_YAML}\n"
                               f"请确保数据已准备好，运行: python3 {SCRIPT_DIR}/prepare_data.py")
    
    # 检查GPU
    has_gpu = check_gpu()
    
    # 根据显存大小和图片尺寸自动调整batch size
    if has_gpu:
        gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1e9
        
        print(f"\n📊 计算最佳batch size...")
        print(f"  - GPU显存: {gpu_memory:.1f} GB")
        print(f"  - 图片尺寸: {IMGSZ}×{IMGSZ}")
        
        # 为800x800图片优化的batch size（RTX 5060 约8GB显存）
        if IMGSZ <= 640:
            # 小图片用大batch
            if gpu_memory > 20:
                batch_size = 64
            elif gpu_memory > 12:
                batch_size = 48
            elif gpu_memory > 8:
                batch_size = 32
            elif gpu_memory > 6:
                batch_size = 24
            else:
                batch_size = 16
        elif IMGSZ <= 800:
            # 中等图片（你的情况）
            if gpu_memory > 20:
                batch_size = 48
            elif gpu_memory > 12:
                batch_size = 32
            elif gpu_memory > 8:   # RTX 5060 8GB
                batch_size = 16  # 推荐起始值
            elif gpu_memory > 6:
                batch_size = 12
            else:
                batch_size = 8
        else:
            # 大图片（>800）
            if gpu_memory > 20:
                batch_size = 32
            elif gpu_memory > 12:
                batch_size = 24
            elif gpu_memory > 8:
                batch_size = 16
            elif gpu_memory > 6:
                batch_size = 12
            else:
                batch_size = 8
        
        print(f"  - 建议batch size: {batch_size}")
        print(f"  ⚠️ 如果出现CUDA out of memory，请手动将batch size调小到12或8")
        
        device = 0
    else:
        batch_size = 4  # CPU用小batch
        device = 'cpu'
    
    print(f"\n📊 最终训练配置:")
    print(f"  - Batch size: {batch_size}")
    print(f"  - 图片尺寸: {IMGSZ}×{IMGSZ}")
    print(f"  - 设备: {'GPU' if device == 0 else 'CPU'}")
    print(f"  - 数据集: {DATA_YAML}")
    print(f"  - AMP自动混合精度: 开启")
    
    # 加载模型
    print(f"\n加载预训练模型: {MODEL_NAME}")
    model = YOLO(MODEL_NAME)
    
    # 开始训练（针对RTX 5060优化）
    print("\n开始训练...")
    results = model.train(
        # 基础配置
        data=str(DATA_YAML),
        epochs=200,
        imgsz=IMGSZ,  # 使用配置的图片尺寸
        
        # GPU优化配置
        device=device,
        batch=batch_size,
        workers=8,  # 数据加载线程数
        
        # 精度优化（RTX 5060支持FP16）
        amp=True,  # 启用自动混合精度（训练更快，显存更省）
        
        # 训练策略优化
        optimizer="auto",  # 自动选择优化器
        cos_lr=True,       # 余弦学习率衰减
        warmup_epochs=3,   # 预热轮数
        warmup_momentum=0.8,
        
        # 数据增强（充分利用GPU性能）
        augment=True,
        mosaic=1.0,        # Mosaic增强
        mixup=0.0,         # 数据量小，暂时关闭mixup
        copy_paste=0.0,    # 数据量小，暂时关闭copy-paste
        
        # 早停策略
        patience=50,       # 50轮没有提升就停止
        
        # 保存配置
        project=str(RESULTS_DIR),
        name="power_panel_detect",
        exist_ok=True,
        save=True,
        save_period=10,    # 每10轮保存一次
        
        # 其他
        pretrained=True,
        verbose=True,
    )
    
    # 保存最佳模型到models目录
    best_model = RESULTS_DIR / "power_panel_detect" / "weights" / "best.pt"
    if best_model.exists():
        shutil.copy(best_model, MODELS_DIR / "best.pt")
        print(f"\n✅ 模型已保存到: {MODELS_DIR / 'best.pt'}")
    
    print(f"\n训练完成！结果保存在: {results.save_dir}")
    return results


def test_detect():
    """测试模型性能（使用相同的图片尺寸）"""
    best_weight = MODELS_DIR / "best.pt"
    
    if not best_weight.exists():
        # 尝试查找最新训练结果
        weights = list(RESULTS_DIR.glob("*/weights/best.pt"))
        if weights:
            best_weight = sorted(weights)[-1]
            print(f"使用最新模型: {best_weight}")
        else:
            raise FileNotFoundError(f"未找到训练权重，请先训练模型")
    
    model = YOLO(str(best_weight))
    print(f"已加载权重: {best_weight}")
    
    # 验证集评估（使用相同的图片尺寸）
    val_results = model.val(
        data=str(DATA_YAML),
        split="val",
        imgsz=IMGSZ,  # 同步使用800
        batch=16,  # 验证时可以用稍大的batch
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
    
    return val_results


def predict_single_image(image_path: str):
    """单张图片预测（使用相同的图片尺寸）"""
    image_path = Path(image_path)
    best_weight = MODELS_DIR / "best.pt"
    
    if not best_weight.exists():
        weights = list(RESULTS_DIR.glob("*/weights/best.pt"))
        if weights:
            best_weight = sorted(weights)[-1]
        else:
            raise FileNotFoundError(f"未找到训练权重，请先训练模型")
    
    if not image_path.exists():
        raise FileNotFoundError(f"未找到图片: {image_path}")
    
    model = YOLO(str(best_weight))
    
    results = model.predict(
        source=str(image_path),
        save=True,
        imgsz=IMGSZ,  # 同步使用800
        conf=0.25,
        device=0 if torch.cuda.is_available() else 'cpu',
    )
    
    # 统计检测结果
    if results[0].boxes is not None:
        power_count = sum(1 for cls in results[0].boxes.cls if int(cls) == 0)
        peg_count = sum(1 for cls in results[0].boxes.cls if int(cls) == 1)
        print(f"\n检测结果:")
        print(f"  - Power Panel: {power_count} 个")
        print(f"  - Peg: {peg_count} 个")
    else:
        print("\n未检测到任何目标")
    
    print(f"\n预测完成，结果已保存到 runs/detect/predict*")
    return results


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="电源板装配检测训练")
    parser.add_argument("--mode", type=str, default="train",
                        choices=["train", "test", "predict"],
                        help="运行模式: train(训练), test(测试), predict(预测)")
    parser.add_argument("--image", type=str, help="预测的图片路径")
    
    args = parser.parse_args()
    
    try:
        if args.mode == "train":
            train_detect()
        elif args.mode == "test":
            test_detect()
        elif args.mode == "predict":
            if not args.image:
                print("请指定 --image 参数")
            else:
                predict_single_image(args.image)
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"错误: {e}")