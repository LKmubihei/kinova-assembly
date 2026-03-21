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
        r = results[0]
        boxes = r.boxes
        annotated = r.plot()

        power_panel_count = 0
        peg_count = 0
        if boxes is not None:
            names = r.names
            for cls in boxes.cls.tolist():
                label = names[int(cls)]
                if label == "power_panel":
                    power_panel_count += 1
                elif label == "peg":
                    peg_count += 1

        # 判断安装状态：1个 power_panel + 4个 peg → installed；无 panel 或 peg 不足4 → not_installed；其他 → error
        if power_panel_count == 1 and peg_count == 4:
            status = "installed"
        elif power_panel_count == 0 or peg_count < 4:
            status = "not_installed"
        else:
            status = "error"

        if save_dir is not None:
            save_dir = Path(save_dir)
            save_dir.mkdir(parents=True, exist_ok=True)
            src = Path(image_path)
            cv2.imwrite(str(save_dir / f"{src.stem}_raw.png"), cv2.imread(str(src)))
            cv2.imwrite(str(save_dir / f"{src.stem}_annotated.png"), annotated)

        return {
            "installed": status == "installed",
            "status": status,
            "power_panel_count": power_panel_count,
            "peg_count": peg_count,
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
