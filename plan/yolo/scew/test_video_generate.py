from ultralytics import YOLO
from pathlib import Path
import cv2
import argparse
import torch

DEVICE = "cuda:0" if torch.cuda.is_available() else "cpu"

DEFAULT_WEIGHT = Path(__file__).resolve().parent / "best.pt"
VIDEO_DIR = Path("/home/pc/kinova_ws/runs/ideo")

FRAME_SKIP = 10  # 每隔 5 帧取一帧进行检测

# 标签显示配置：peg_pcb 跳过，zhun/buzhun 自定义颜色和文字
_LABEL_CONFIG = {
    "peg_pcb": None,  # None 表示跳过不绘制
    "zhun":    {"color": (0, 200, 0),   "text": "Aligned"},
    "buzhun":  {"color": (0, 0, 220),   "text": "Not aligned"},
}


def _draw_custom(frame: "cv2.Mat", result) -> "cv2.Mat":
    """在原始帧上按自定义规则绘制检测框，跳过 peg_pcb。"""
    img = frame.copy()
    if result.boxes is None:
        return img

    names = result.names
    for box, cls, conf in zip(
        result.boxes.xyxy.tolist(),
        result.boxes.cls.tolist(),
        result.boxes.conf.tolist(),
    ):
        label = names[int(cls)]
        cfg = _LABEL_CONFIG.get(label)

        if cfg is None:
            continue  # 跳过 peg_pcb 及未配置类别

        x1, y1, x2, y2 = map(int, box)
        color = cfg["color"]
        display_text = f"{cfg['text']} {conf:.2f}"

        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        text_y = max(y1 - 6, 14)
        cv2.putText(img, display_text, (x1, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

    return img


def predict_video(
    video_path: str,
    weight: str = str(DEFAULT_WEIGHT),
    save: bool = True,
    show: bool = False,
    frame_skip: int = FRAME_SKIP,
):
    """对视频文件按指定间隔抽帧进行 YOLO 检测。

    Args:
        video_path:  输入视频路径。
        weight:      模型权重路径，默认使用同目录下的 best.pt。
        save:        是否将检测结果视频保存到 runs/detect/predict_video/。
        show:        是否实时显示检测画面（需要图形界面）。
        frame_skip:  每隔多少帧取一帧，默认 5（降低 5 倍帧率）。
    """
    weight_path = Path(weight)
    video_path = Path(video_path)

    if not weight_path.exists():
        raise FileNotFoundError(f"未找到模型权重: {weight_path}")
    if not video_path.exists():
        raise FileNotFoundError(f"未找到视频文件: {video_path}")

    model = YOLO(str(weight_path))
    model.to(DEVICE)
    print(f"使用设备: {DEVICE}")

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    out_writer = None
    if save:
        out_dir = Path("runs/detect/predict_video")
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / video_path.name
        out_fps = max(1.0, fps / frame_skip)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out_writer = cv2.VideoWriter(str(out_path), fourcc, out_fps, (width, height))

    raw_idx = 0
    processed = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        raw_idx += 1
        if (raw_idx - 1) % frame_skip != 0:
            continue

        processed += 1
        results = model.predict(source=frame, imgsz=1280, verbose=False, device=DEVICE)
        result = results[0]

        boxes = result.boxes
        n_det = len(boxes) if boxes is not None else 0
        print(f"Frame {raw_idx:05d}/{total_frames} (processed {processed:04d}): {n_det} detections")

        annotated = _draw_custom(frame, result)

        if show:
            cv2.imshow(f"YOLO - {video_path.name}", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        if out_writer is not None:
            out_writer.write(annotated)

    cap.release()
    if out_writer is not None:
        out_writer.release()
    if show:
        cv2.destroyAllWindows()

    print(f"\n视频检测完成：原始 {raw_idx} 帧，实际处理 {processed} 帧（每 {frame_skip} 帧取一帧）。")
    if save:
        print(f"检测结果已保存到: {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLO 视频批量检测")
    parser.add_argument("--video_dir", default=str(VIDEO_DIR), help="视频文件夹路径")
    parser.add_argument("--weight", default=str(DEFAULT_WEIGHT), help="模型权重路径")
    parser.add_argument("--save", action="store_true", default=True, help="保存结果视频")
    parser.add_argument("--show", action="store_true", default=False, help="实时显示检测画面")
    args = parser.parse_args()

    video_dir = Path(args.video_dir)
    videos = sorted(video_dir.glob("*.mp4"))
    if not videos:
        print(f"未在 {video_dir} 找到 .mp4 文件")
    else:
        for vp in videos:
            print(f"\n{'='*50}\n处理视频: {vp.name}\n{'='*50}")
            predict_video(str(vp), weight=args.weight, save=args.save, show=args.show)
