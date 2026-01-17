from pathlib import Path

from ultralytics import YOLO
from ultralytics.cfg import get_cfg
from ultralytics.models.yolo.detect.predict import DetectionPredictor
from ultralytics.utils import DEFAULT_CFG


def main():
    cfg = get_cfg(DEFAULT_CFG)

    cfg.task = "detect"
    cfg.mode = "predict"

    cfg.model = "yolo26s.pt"
    cfg.source = 0          # change to 1 or 2 if needed
    cfg.show = True
    cfg.save = True
    cfg.conf = 0.25
    cfg.iou = 0.7
    cfg.imgsz = 640
    cfg.device = None
    cfg.stream = False
    cfg.verbose = True
    cfg.classes = [0] 
    cfg.project = Path("runs/detect")
    cfg.name = "predict"
    cfg.exist_ok = True
    cfg.save_txt = True
    cfg.save_crop = True


    # Create YOLO model (same as CLI)
    yolo = YOLO(cfg.model, task=cfg.task)

    # Create predictor
    predictor = DetectionPredictor(cfg)

    # Attach torch model ONLY
    predictor.setup_model(yolo.model)

    # DO NOT call setup_source()
    # CLI handles this internally

    predictor.predict_cli()


if __name__ == "__main__":
    main()
