"""
Step 1 - 料箱检测
检测料箱中是否存在 power_panel，使用 bin_pcb_detection 模型
"""

from pathlib import Path
import cv2
from ultralytics import YOLO

_MODEL_PATH = Path(__file__).parent.parent / "yolo/bin_pcb_detection/models/bin_model.pt"


class BinDetector:
    def __init__(self, model_path=None, conf=0.7):
        path = model_path or _MODEL_PATH
        self.model = YOLO(str(path))
        self.conf = conf

    def detect(self, image_path, save_dir=None) -> dict:
        """
        检测料箱图像，返回检测结果。

        Args:
            image_path: 输入图像路径
            save_dir:   若指定，保存原图和标注图到该目录

        Returns:
            {
                "found": bool,
                "count": int,
                "detections": list,
                "annotated": np.ndarray  # 标注后图像
            }
        """
        results = self.model(str(image_path), conf=self.conf, verbose=False)
        boxes = results[0].boxes
        count = len(boxes) if boxes is not None else 0
        annotated = results[0].plot()

        if save_dir is not None:
            save_dir = Path(save_dir)
            save_dir.mkdir(parents=True, exist_ok=True)
            src = Path(image_path)
            cv2.imwrite(str(save_dir / f"{src.stem}_raw.png"), cv2.imread(str(src)))
            cv2.imwrite(str(save_dir / f"{src.stem}_annotated.png"), annotated)

        return {
            "found": count > 0,
            "count": count,
            "detections": boxes.data.tolist() if boxes is not None else [],
            "annotated": annotated,
        }


if __name__ == "__main__":
    import sys
    det = BinDetector()
    # img = sys.argv[1] if len(sys.argv) > 1 else str(
    #     Path(__file__).parent.parent / "yolo/step1.png"
    # )
    # result = det.detect(img, save_dir="/tmp/bin_detect")
    # print(f"found={result['found']}  count={result['count']}")
    img = '/home/pc/kinova_ws/src/plan/yolo/step1.png'
    result = det.detect(img, save_dir="/home/pc/kinova_ws/src/plan/yolo/step1_out.png")
    print(f"found={result['found']}  count={result['count']}")

