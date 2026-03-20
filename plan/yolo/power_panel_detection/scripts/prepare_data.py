#!/usr/bin/env python3
"""
数据预处理脚本 - 电源板装配检测
检测目标: power_panel (电源板), peg (定位柱)
"""

import json
import random
import shutil
from pathlib import Path
import cv2

# ==================== 配置区域 ====================
# 原始数据目录（你的110个已标注数据）
RAW_DATA_DIR = Path("/home/pb/Hisense_pcb_inspection/src/Hisense_assemble-main/data/电源组件")

# 新项目目录
PROJECT_DIR = Path("/home/pb/Hisense_pcb_inspection/src/Hisense_assemble-main/power_panel_detection")
OUTPUT_DIR = PROJECT_DIR / "data"

# 类别映射（两个类别）
CLASS_MAP = {
    "power_panel": 0,  # 电源板
    "peg": 1,          # 定位柱
}

# 训练集比例
TRAIN_RATIO = 0.8
RANDOM_SEED = 42
# =================================================

def _find_image_for_json(json_path: Path) -> Path:
    """根据JSON文件查找对应的图片"""
    # 尝试常见的图片扩展名
    stem = json_path.stem
    for ext in ['.png', '.jpg', '.jpeg']:
        img_path = json_path.parent / f"{stem}{ext}"
        if img_path.exists():
            return img_path
    raise FileNotFoundError(f"找不到图片: {json_path}")

def _shape_to_yolo_line(shape: dict, img_w: float, img_h: float) -> str | None:
    """将单个Labelme形状转换为YOLO格式"""
    label = shape.get("label")
    if label not in CLASS_MAP:
        return None

    points = shape.get("points", [])
    if len(points) < 2:
        return None

    # 计算边界框
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x1, x2 = min(xs), max(xs)
    y1, y2 = min(ys), max(ys)

    # 计算宽高
    width = max(0.0, x2 - x1)
    height = max(0.0, y2 - y1)
    if width <= 0 or height <= 0:
        return None

    # 计算YOLO格式
    x_center = (x1 + x2) / 2.0 / img_w
    y_center = (y1 + y2) / 2.0 / img_h
    width /= img_w
    height /= img_h

    class_id = CLASS_MAP[label]
    return f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}"

def prepare_dataset():
    """准备数据集"""
    print("=" * 60)
    print("🔧 电源板装配检测 - 数据预处理")
    print("=" * 60)
    
    # 创建输出目录
    for split in ['train', 'val']:
        (OUTPUT_DIR / 'images' / split).mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / 'labels' / split).mkdir(parents=True, exist_ok=True)
    
    # 收集所有数据
    all_items = []
    json_files = sorted(RAW_DATA_DIR.glob("*.json"))
    
    print(f"\n📂 扫描原始数据: {RAW_DATA_DIR}")
    print(f"找到 {len(json_files)} 个标注文件")
    
    for json_path in json_files:
        try:
            img_path = _find_image_for_json(json_path)
            
            # 读取JSON统计信息
            with open(json_path, 'r') as f:
                data = json.load(f)
            
            # 统计各类别数量
            power_panel_count = 0
            peg_count = 0
            for shape in data.get('shapes', []):
                label = shape.get('label')
                if label == 'power_panel':
                    power_panel_count += 1
                elif label == 'peg':
                    peg_count += 1
            
            all_items.append({
                'json': json_path,
                'img': img_path,
                'stem': img_path.stem,
                'power_panel': power_panel_count,
                'peg': peg_count
            })
            
        except Exception as e:
            print(f"  ⚠️ 跳过 {json_path.name}: {e}")
    
    print(f"\n📊 数据统计:")
    print(f"  有效数据对: {len(all_items)} 个")
    
    # 随机打乱
    random.seed(RANDOM_SEED)
    random.shuffle(all_items)
    
    # 划分训练/验证集
    split_idx = int(len(all_items) * TRAIN_RATIO)
    train_items = all_items[:split_idx]
    val_items = all_items[split_idx:]
    
    print(f"\n📊 数据集划分:")
    print(f"  训练集: {len(train_items)} 张")
    print(f"  验证集: {len(val_items)} 张")
    
    # 处理数据
    def process_items(items, split_name):
        processed = 0
        total_power = 0
        total_peg = 0
        
        for item in items:
            try:
                # 读取图片获取尺寸
                img = cv2.imread(str(item['img']))
                if img is None:
                    raise ValueError(f"无法读取图片: {item['img']}")
                img_h, img_w = img.shape[:2]
                
                # 读取JSON并转换
                with open(item['json'], 'r') as f:
                    data = json.load(f)
                
                # 转换标注
                lines = []
                for shape in data.get('shapes', []):
                    line = _shape_to_yolo_line(shape, img_w, img_h)
                    if line is not None:
                        lines.append(line)
                
                # 复制图片
                img_dst = OUTPUT_DIR / 'images' / split_name / item['img'].name
                shutil.copy2(item['img'], img_dst)
                
                # 保存标注
                label_dst = OUTPUT_DIR / 'labels' / split_name / f"{item['stem']}.txt"
                with open(label_dst, 'w') as f:
                    f.write('\n'.join(lines))
                
                processed += 1
                total_power += item['power_panel']
                total_peg += item['peg']
                
            except Exception as e:
                print(f"  ⚠️ 处理失败 {item['img'].name}: {e}")
        
        return processed, total_power, total_peg
    
    print("\n📋 开始处理训练集...")
    tr_ok, tr_power, tr_peg = process_items(train_items, 'train')
    
    print("\n📋 开始处理验证集...")
    val_ok, val_power, val_peg = process_items(val_items, 'val')
    
    # 创建data.yaml
    data_yaml = OUTPUT_DIR / 'data.yaml'
    yaml_content = f"""
# 电源板装配检测数据集
path: {OUTPUT_DIR.absolute()}  # 数据集根目录
train: images/train  # 训练图片
val: images/val      # 验证图片

# 类别信息
nc: 2
names: ['power_panel', 'peg']

# 统计信息
total_images: {len(all_items)}
train_images: {tr_ok}
val_images: {val_ok}
total_power_panels: {tr_power + val_power}
total_pegs: {tr_peg + val_peg}
"""
    with open(data_yaml, 'w') as f:
        f.write(yaml_content.strip())
    
    print("\n" + "=" * 60)
    print("✅ 数据预处理完成！")
    print("=" * 60)
    print(f"📁 数据集位置: {OUTPUT_DIR}")
    print(f"📄 配置文件: {data_yaml}")
    print(f"\n📊 最终统计:")
    print(f"  训练集: {tr_ok} 张")
    print(f"    - power_panel: {tr_power} 个")
    print(f"    - peg: {tr_peg} 个")
    print(f"  验证集: {val_ok} 张")
    print(f"    - power_panel: {val_power} 个")
    print(f"    - peg: {val_peg} 个")

if __name__ == "__main__":
    prepare_dataset()