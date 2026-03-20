# Hisense PCB/Power Panel Inspection

此项目包含两个主要检测子系统：

1. `bin_pcb_detection`：物料箱PCB目标检测与计数
2. `power_panel_detection`：电源板装配状态检测

---

## 目录结构

- `bin_pcb_detection/`
  - `data/`：原始图像与标签
  - `models/`：预训练与训练生成模型
  - `results/`：推理结果 (`inference`, `bin_pcb_detect` 等)
  - `scripts/`
    - `prepare_bin_data.py`：数据预处理与标签格式转换
    - `train_bin.py`：训练/测试/单图预测逻辑
    - `test.py`：与训练脚本类似，配合测试集使用
    - `count_pcb.py`：仿 `detect.py` 风格接口（单图、批量、实时）用于计数

- `power_panel_detection/`
  - `data/`：原始图像、标签与配置
  - `models/`：预训练与训练生成模型
  - `results/`：推理结果
  - `scripts/`
    - `prepare_data.py`：电源板数据预处理与 `data.yaml` 生成
    - `train.py`：训练/测试/单图预测（模型处理类 `power_panel`, `peg`）
    - `detect.py`：单图/批量/流式检测

- `requirements.txt`：依赖包列表（强烈建议使用虚拟环境）
- `yolo26n.pt`：基础 YOLOv8n 权重

---

## 快速启动

### 1. 进入项目目录

```bash
cd /home/pb/Hisense_pcb_inspection/src/Hisense_assemble-main
```

### 2. 创建并激活 Python 3 虚拟环境（可选但推荐）

```bash
python3 -m venv Hisense_venv
source Hisense_venv/bin/activate
```

### 3. 安装依赖

```bash
pip install -r requirements.txt
```

若环境中已有 ROS2 / 机器人中间件依赖，直接使用本仓库的已有环境 `Hisense_venv` 即可。

---

## 子系统一：物料箱PCB检测与计数（`bin_pcb_detection`）

### 1) 数据准备

- 使用 `prepare_bin_data.py` 将 `data` 下标签转为训练格式，生成 `data.yaml`（已存在）

```bash
python3 bin_pcb_detection/scripts/prepare_bin_data.py
```

### 2) 训练模型

```bash
python3 bin_pcb_detection/scripts/train_bin.py --mode train
```

- 训练结果保存到 `runs/train/*`
- 参考 `models/bin_model.pt`、`models/bin_model_v800.pt` 已训练权重

### 3) 测试/验证

```bash
python3 bin_pcb_detection/scripts/train_bin.py --mode test
```

- 默认选用 `data.yaml` 里配置的数据集（train/val）
- 输出指标（mAP, Precision, Recall）到终端

### 4) 单图预测（计数）

```bash
python3 bin_pcb_detection/scripts/train_bin.py --mode predict --image path/to/image.jpg
```

- 输出 PCB 数量与置信度列表
- 结果保存到 `runs/detect/predict*`

### 5) 批量/单图/流式计数（`count_pcb.py`）

```bash
python3 bin_pcb_detection/scripts/count_pcb.py --mode single --image path/to/image.jpg
python3 bin_pcb_detection/scripts/count_pcb.py --mode batch --dir path/to/images
python3 bin_pcb_detection/scripts/count_pcb.py --mode stream --stream 0
```

- 兼容 `--source`：可传文件、目录或 `camera_id`（如 `0`）
- 返回图像标注结果、物料计数、累计统计
- `--conf` 可调整置信度阈值

示例：

```bash
python3 bin_pcb_detection/scripts/count_pcb.py --source /your/img/dir --output runs/inference --conf 0.25
```

---

## 子系统二：电源板装配检测（`power_panel_detection`）

### 1) 数据准备

```bash
python3 power_panel_detection/scripts/prepare_data.py
```

- 生成 `data.yaml` 并拆分训练/验证
- 统计 `power_panel` 与 `peg` 目标数量

### 2) 训练模型

```bash
python3 power_panel_detection/scripts/train.py --mode train
```

- 模型文件通常生成至 `runs/train/*`

### 3) 测试模型

```bash
python3 power_panel_detection/scripts/train.py --mode test
```

### 4) 单图预测

```bash
python3 power_panel_detection/scripts/train.py --mode predict --image path/to/image.jpg
```

- 输出 `power_panel` / `peg` 计数
- 保存预测图至 `runs/detect/predict*`

### 5) 通用检测接口

```bash
python3 power_panel_detection/scripts/detect.py --image test.jpg
python3 power_panel_detection/scripts/detect.py --dir ./images
python3 power_panel_detection/scripts/detect.py --stream       # 新增：实时相机检测（默认摄像头0）
python3 power_panel_detection/scripts/detect.py --stream 0     # 指定摄像头ID
python3 power_panel_detection/scripts/detect.py --stream 1     # 指定其他摄像头
python3 power_panel_detection/scripts/detect.py --stream video.mp4  # 本地视频回放
```

- `--stream`：支持 `int`（摄像头编号）、`str`（视频文件路径）、`None`（默认 0）
- `--output`：设置推理输出目录，默认 `results/inference`

---

## 常用参数说明（全局）

- `--mode`: `train/test/predict`（说明见各脚本）
- `--image`: 处理单张图片（预测场景）
- `--dir`: 处理目录内所有图片（批量推理）
- `--stream`: 摄像头/视频流（实时检测）
- `--source`: 通用路径输入（兼容 detect 风格）
- `--output`: 推理结果目录
- `--conf`: 置信阈值（例如 0.1~0.5）

## 新增功能：相机实时检测

`power_panel_detection/scripts/detect.py` 已支持完整的相机实时检测，包含：
- 自动检测摄像头分辨率/帧率并切换到最优值
- 实时检测 `power_panel` 和 `peg`，画面叠加框与状态
- 按键交互：`q` 退出，`s` 保存当前帧，`i` 显示摄像头参数，`r` 重置分辨率
- 输出结果在 `results/inference/stream_captures` 中保存截图

使用示例：
```bash
python3 power_panel_detection/scripts/detect.py --stream
python3 power_panel_detection/scripts/detect.py --stream 0
python3 power_panel_detection/scripts/detect.py --stream /path/to/your_video.mp4
```

- 需要保证 `opencv-python` 和 `ultralytics` 已安装; 若未安装请先：
```bash
pip install -r requirements.txt
```
/home/pc/kinova_ws/src/plan 关注这个，我需要执行的是一个规划任务。首先拿从料箱bin中拿取 power_panel，需要确保其中有 power_panel（通过yolo 检测 /home/pc/kinova_ws/src/plan/yolo/bin_pcb_detection），然后把 power_panel 安装到背板上去，使用 /home/pc/kinova_ws/src/plan/yolo/power_panel_detection 检测有没有装好。 之后使用 /home/pc/kinova_ws/src/plan/yolo/scew 检测 螺丝孔位置进行安装 /home/pc/kinova_ws/src/plan/yolo/scew。
我已经训练好所有的yolo 模型，测试的图像也分别在 step1  step2 step3.png  。 但是目前路径很乱，我希望整理一下  /home/pc/kinova_ws/src/plan 这个大项目， 先用 htn 实现一版 规划 (可以先用pddl 格式)