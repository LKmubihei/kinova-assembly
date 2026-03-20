#!/usr/bin/env python3
"""
训练脚本 - 物料箱PCB检测
针对RTX 5060 (Blackwell架构) 优化
图片尺寸: 800x800
"""

from pathlib import Path
from ultralytics import YOLO
import torch
import shutil

# ==================== 配置区域 ====================
# 获取当前脚本所在目录的绝对路径
SCRIPT_DIR = Path(__file__).resolve().parent
# 项目根目录 (scripts的上一级)
PROJECT_ROOT = SCRIPT_DIR.parent.parent  # 注意：这里需要多上一级

# 数据集配置文件 - 修正路径
DATA_YAML = PROJECT_ROOT / "bin_pcb_detection" / "bin_data.yaml"

# 结果保存目录
RESULTS_DIR = PROJECT_ROOT / "bin_pcb_detection" / "results"

# 模型保存目录
MODELS_DIR = PROJECT_ROOT / "bin_pcb_detection" / "models"

# 预训练模型路径（优先使用本地模型）
LOCAL_MODEL = MODELS_DIR / "yolo11n.pt"
if LOCAL_MODEL.exists():
    MODEL_NAME = str(LOCAL_MODEL)
else:
    MODEL_NAME = "yolo11n.pt"  # 如果本地没有，从网上下载

# 图片尺寸设置
IMGSZ = 800  # 你的图片尺寸

# RTX 5060 优化配置
import os
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
    
    # 检查是否为Blackwell架构 (sm_120)
    arch_list = torch.cuda.get_arch_list()
    if any('sm_120' in arch for arch in arch_list):
        print("✅ 当前PyTorch支持sm_120，可以充分发挥RTX 5060性能！")
    else:
        print("⚠️ 当前PyTorch不支持sm_120，性能会受限")
    
    return True


def ensure_dirs():
    """确保所有必要的目录存在"""
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    print(f"📁 结果保存目录: {RESULTS_DIR}")
    print(f"📁 模型保存目录: {MODELS_DIR}")


def calculate_batch_size(gpu_memory: float) -> int:
    """
    根据显存大小和图片尺寸计算合适的batch size
    """
    # 为800x800图片优化的batch size（RTX 5060 约8GB显存）
    if IMGSZ <= 640:
        # 小图片用大batch
        if gpu_memory > 20:
            return 64
        elif gpu_memory > 12:
            return 48
        elif gpu_memory > 8:
            return 32
        elif gpu_memory > 6:
            return 24
        else:
            return 16
    elif IMGSZ <= 800:
        # 中等图片（你的情况）
        if gpu_memory > 20:
            return 48
        elif gpu_memory > 12:
            return 32
        elif gpu_memory > 8:   # RTX 5060 8GB
            return 16  # 推荐起始值
        elif gpu_memory > 6:
            return 12
        else:
            return 8
    else:
        # 大图片（>800）
        if gpu_memory > 20:
            return 32
        elif gpu_memory > 12:
            return 24
        elif gpu_memory > 8:
            return 16
        elif gpu_memory > 6:
            return 12
        else:
            return 8


def train_detect():
    """训练YOLO目标检测模型（针对RTX 5060优化）"""
    
    # 确保目录存在
    ensure_dirs()
    
    # 检查配置文件
    if not DATA_YAML.exists():
        raise FileNotFoundError(
            f"未找到数据集配置文件: {DATA_YAML}\n"
            f"请先运行数据预处理脚本: python3 {SCRIPT_DIR}/prepare_bin_data.py"
        )

    # 检查GPU
    has_gpu = check_gpu()
    
    # 根据显存大小自动调整batch size
    if has_gpu:
        gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1e9
        batch_size = calculate_batch_size(gpu_memory)
        device = 0
        print(f"\n📊 根据显存 ({gpu_memory:.1f}GB) 和图片尺寸 ({IMGSZ}) 选择 batch size: {batch_size}")
    else:
        batch_size = 4  # CPU用小batch
        device = 'cpu'
        print(f"\n📊 CPU训练，batch size: {batch_size}")

    # 加载模型
    print(f"\n加载预训练模型: {MODEL_NAME}")
    model = YOLO(MODEL_NAME)

    # 开始训练
    print("\n开始训练...")
    results = model.train(
        # 基础配置
        data=str(DATA_YAML),
        epochs=200,
        imgsz=IMGSZ,
        
        # GPU优化配置
        device=device,
        batch=batch_size,
        workers=8,
        
        # 精度优化
        amp=True,
        
        # 训练策略
        optimizer="auto",
        cos_lr=True,
        warmup_epochs=3,
        warmup_momentum=0.8,
        
        # 数据增强
        augment=True,
        mosaic=1.0,
        
        # 早停策略
        patience=50,
        
        # 保存配置
        project=str(RESULTS_DIR),
        name="bin_pcb_detect",  # 这里指定了文件夹名
        exist_ok=True,
        save=True,
        save_period=10,
        
        # 其他
        pretrained=True,
        verbose=True,
    )

    # 保存最佳模型到models目录
    best_model = RESULTS_DIR / "bin_pcb_detect" / "weights" / "best.pt"  # ✅ 注意这里是 best.pt
    if best_model.exists():
        # 确保 models 目录存在
        MODELS_DIR.mkdir(parents=True, exist_ok=True)
        
        # 复制为 bin_model.pt
        model_path = MODELS_DIR / "bin_model.pt"
        shutil.copy(best_model, model_path)
        print(f"\n✅ 模型已保存到: {model_path}")
        
        # 也保存一个带版本号的备份
        backup_path = MODELS_DIR / f"bin_model_v{IMGSZ}.pt"
        shutil.copy(best_model, backup_path)
        print(f"✅ 备份模型已保存到: {backup_path}")
    else:
        print(f"\n⚠️ 未找到模型文件: {best_model}")
        print(f"请检查目录: {RESULTS_DIR / 'bin_pcb_detect' / 'weights'}")

    print(f"\n训练完成！结果保存在: {results.save_dir}")
    return results


def test_detect():
    """测试模型性能"""
    # 查找最佳权重
    best_weight = MODELS_DIR / "bin_model.pt"
    
    if not best_weight.exists():
        # 尝试查找最新的训练结果
        weights = list(RESULTS_DIR.glob("*/weights/bin_model.pt"))
        if weights:
            best_weight = sorted(weights)[-1]
            print(f"使用最新模型: {best_weight}")
        else:
            raise FileNotFoundError(
                f"未找到训练权重，请先训练模型: python3 {SCRIPT_DIR}/train_bin.py"
            )

    # 加载模型
    model = YOLO(str(best_weight))
    print(f"已加载权重: {best_weight}")

    # 验证集评估（使用相同的图片尺寸）
    val_results = model.val(
        data=str(DATA_YAML),
        split="val",
        imgsz=IMGSZ,
        batch=16,
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
    """单张图片预测"""
    image_path = Path(image_path)
    
    # 查找最佳权重
    best_weight = MODELS_DIR / "bin_model.pt"
    
    if not best_weight.exists():
        weights = list(RESULTS_DIR.glob("*/weights/bin_model.pt"))
        if weights:
            best_weight = sorted(weights)[-1]
        else:
            raise FileNotFoundError(
                f"未找到训练权重，请先训练模型: python3 {SCRIPT_DIR}/train_bin.py"
            )
    
    if not image_path.exists():
        raise FileNotFoundError(f"未找到图片: {image_path}")

    # 加载模型
    model = YOLO(str(best_weight))
    print(f"已加载权重: {best_weight}")

    # 预测（使用相同的图片尺寸）
    results = model.predict(
        source=str(image_path),
        save=True,
        imgsz=IMGSZ,
        conf=0.25,
        device=0 if torch.cuda.is_available() else 'cpu',
    )

    # 统计PCB数量
    if results[0].boxes is not None:
        pcb_count = len(results[0].boxes)
        print(f"\n检测结果:")
        print(f"  - PCB数量: {pcb_count} 个")
        confs = [f"{c:.2f}" for c in results[0].boxes.conf]
        print(f"  - 置信度: {confs}")
    else:
        print("\n未检测到PCB")

    print(f"\n预测完成，结果已保存到 runs/detect/predict*")
    return results


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="物料箱PCB检测训练")
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