import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from ultralytics import YOLO

device = 0 if torch.cuda.is_available() else "cpu"
model = YOLO("yolo11n.pt")

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        image = np.asanyarray(color_frame.get_data())

        results = model.predict(
            source=image,
            device=device,
            conf=0.5,
            verbose=False,
        )

        annotated = results[0].plot()
        cv2.imshow("YOLO11 RealSense", annotated)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
