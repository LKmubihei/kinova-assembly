"""
Step 2 - 背板安装验证
检测 power_panel 是否已正确安装到背板，使用 power_panel_detection 模型
"""

from pathlib import Path
import cv2
from ultralytics import YOLO

_MODEL_PATH = Path(__file__).parent.parent / "yolo/power_panel_detection/models/best.pt"


class PanelDetector:
    def __init__(self, model_path=None, conf=0.25):
        path = model_path or _MODEL_PATH
        self.model = YOLO(str(path))
        self.conf = conf

    def detect(self, image_path, save_dir=None) -> dict:
        """
        检测背板图像，判断 power_panel 是否安装到位。

        Args:
            image_path: 输入图像路径
            save_dir:   若指定，保存原图和标注图到该目录

        Returns:
            {
                "installed": bool,
                "count": int,
                "detections": list,
                "annotated": np.ndarray
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
            "installed": count > 0,
            "count": count,
            "detections": boxes.data.tolist() if boxes is not None else [],
            "annotated": annotated,
        }


if __name__ == "__main__":
    import sys
    img = sys.argv[1] if len(sys.argv) > 1 else str(
        Path(__file__).parent.parent / "yolo/step2.png"
    )
    det = PanelDetector()
    result = det.detect(img, save_dir="/tmp/panel_detect")
    print(f"installed={result['installed']}  count={result['count']}")

