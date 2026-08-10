import time
from collections import deque

import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from ultralytics import YOLO

# =========================
# 설정
# =========================
MODEL_PATH = "yolo11n.pt"     # 전체 파이프라인 완성 후 yolo11x.pt로 변경
TARGET_CLASS = "bottle"
CONF_THRESHOLD = 0.50

WIDTH = 640
HEIGHT = 480
FPS = 30

HISTORY_SIZE = 10
LOST_RESET_FRAMES = 15       # 이 프레임 동안 물체가 없으면 smoothing 초기화

MIN_DEPTH_M = 0.10
MAX_DEPTH_M = 2.00

# =========================
# 모델 설정
# =========================
device = 0 if torch.cuda.is_available() else "cpu"
model = YOLO(MODEL_PATH)

print(f"YOLO model: {MODEL_PATH}")
print(f"Device: {torch.cuda.get_device_name(0) if device == 0 else 'CPU'}")

# =========================
# 좌표 smoothing
# =========================
x_history = deque(maxlen=HISTORY_SIZE)
y_history = deque(maxlen=HISTORY_SIZE)
z_history = deque(maxlen=HISTORY_SIZE)

lost_frames = 0

def clear_history():
    x_history.clear()
    y_history.clear()
    z_history.clear()

# =========================
# RealSense 설정
# =========================
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    WIDTH,
    HEIGHT,
    rs.format.bgr8,
    FPS,
)

config.enable_stream(
    rs.stream.depth,
    WIDTH,
    HEIGHT,
    rs.format.z16,
    FPS,
)

profile = pipeline.start(config)
align = rs.align(rs.stream.color)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

print(f"Depth scale: {depth_scale}")
print(f"'{TARGET_CLASS}' 검출을 시작합니다. 종료: q")

prev_time = time.time()

try:
    while True:
        frames = pipeline.wait_for_frames()

        # Depth를 RGB 좌표계에 정렬
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        image_h, image_w = color_image.shape[:2]

        # 정렬된 depth frame의 intrinsic 사용
        intrinsics = (
            depth_frame.profile
            .as_video_stream_profile()
            .get_intrinsics()
        )

        # 단일 이미지이므로 stream=True 불필요
        result = model.predict(
            source=color_image,
            device=device,
            conf=CONF_THRESHOLD,
            imgsz=640,
            verbose=False,
        )[0]

        candidates = []

        for box in result.boxes:
            class_id = int(box.cls.item())
            class_name = model.names[class_id]
            confidence = float(box.conf.item())

            if class_name != TARGET_CLASS:
                continue

            x1, y1, x2, y2 = map(
                int,
                box.xyxy[0].cpu().numpy(),
            )

            candidates.append(
                {
                    "confidence": confidence,
                    "bbox": (x1, y1, x2, y2),
                }
            )

        # 가장 confidence가 높은 물체 하나만 사용
        if candidates:
            target = max(
                candidates,
                key=lambda item: item["confidence"],
            )

            confidence = target["confidence"]
            x1, y1, x2, y2 = target["bbox"]

            x1 = np.clip(x1, 0, image_w - 1)
            x2 = np.clip(x2, 0, image_w - 1)
            y1 = np.clip(y1, 0, image_h - 1)
            y2 = np.clip(y2, 0, image_h - 1)

            ux = (x1 + x2) // 2
            uy = (y1 + y2) // 2

            bbox_w = max(x2 - x1, 1)
            bbox_h = max(y2 - y1, 1)

            # 박스 중앙 약 10% 면적:
            # 한 변을 약 sqrt(0.1) = 0.316배 사용
            roi_w = max(int(bbox_w * 0.316), 5)
            roi_h = max(int(bbox_h * 0.316), 5)

            roi_x1 = max(0, ux - roi_w // 2)
            roi_x2 = min(image_w, ux + roi_w // 2 + 1)
            roi_y1 = max(0, uy - roi_h // 2)
            roi_y2 = min(image_h, uy + roi_h // 2 + 1)

            depth_roi_raw = depth_image[
                roi_y1:roi_y2,
                roi_x1:roi_x2,
            ]

            depth_roi_m = (
                depth_roi_raw.astype(np.float32)
                * depth_scale
            )

            valid_depths = depth_roi_m[
                (depth_roi_m >= MIN_DEPTH_M)
                & (depth_roi_m <= MAX_DEPTH_M)
            ]

            if valid_depths.size > 0:
                z_m = float(np.median(valid_depths))

                point = rs.rs2_deproject_pixel_to_point(
                    intrinsics,
                    [float(ux), float(uy)],
                    z_m,
                )

                x_m, y_m, z_m = map(float, point)

                x_history.append(x_m)
                y_history.append(y_m)
                z_history.append(z_m)

                smooth_x = float(np.median(x_history))
                smooth_y = float(np.median(y_history))
                smooth_z = float(np.median(z_history))

                lost_frames = 0

                # cm 단위 표시
                tx = smooth_x * 100
                ty = smooth_y * 100
                tz = smooth_z * 100

                cv2.rectangle(
                    color_image,
                    (x1, y1),
                    (x2, y2),
                    (255, 0, 0),
                    2,
                )

                cv2.rectangle(
                    color_image,
                    (roi_x1, roi_y1),
                    (roi_x2, roi_y2),
                    (0, 255, 255),
                    1,
                )

                cv2.circle(
                    color_image,
                    (ux, uy),
                    4,
                    (0, 0, 255),
                    -1,
                )

                text_y = max(y1 - 10, 25)

                cv2.putText(
                    color_image,
                    (
                        f"{TARGET_CLASS} {confidence:.2f} | "
                        f"XYZ: {tx:.1f}, {ty:.1f}, {tz:.1f} cm"
                    ),
                    (x1, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

                print(
                    f"\rCamera XYZ: "
                    f"{smooth_x:+.3f}, "
                    f"{smooth_y:+.3f}, "
                    f"{smooth_z:+.3f} m",
                    end="",
                    flush=True,
                )

            else:
                lost_frames += 1

        else:
            lost_frames += 1

        # 일정 시간 검출이 없으면 이전 좌표 제거
        if lost_frames >= LOST_RESET_FRAMES:
            clear_history()
            lost_frames = 0

        # 실제 처리 FPS
        current_time = time.time()
        fps_value = 1.0 / max(current_time - prev_time, 1e-6)
        prev_time = current_time

        cv2.putText(
            color_image,
            f"FPS: {fps_value:.1f}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        cv2.imshow("RealSense YOLO 3D Detection", color_image)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    print("\n종료합니다.")
    pipeline.stop()
    cv2.destroyAllWindows()
