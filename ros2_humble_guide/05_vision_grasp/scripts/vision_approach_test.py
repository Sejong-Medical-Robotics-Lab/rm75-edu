"""
[05 테스트 B] 비전 좌표로 접근 — 파지는 하지 않는다

A단계에서 검증한 좌표 파이프라인으로 물병 위치를 얻고,
그 앞 APPROACH_BACK 만큼 떨어진 지점까지 MoveJ_P로 이동한다.

기하 (모두 base_link 기준, 측면 파지 자세에서 그리퍼는 +x를 향한다):

    물병 중심 x  ← 비전 (반지름 보정 포함)
    물병 앞면 x  = 중심 x - 반지름
    파지 시 Link7 x = 앞면 x - GRASP_OFFSET
    접근 시 Link7 x = 파지 x - APPROACH_BACK

GRASP_OFFSET(0.098)은 실측값이다 — 드래그 티칭으로 물병을 감싼 상태에서
tf2_echo base_link Link7 을 읽어 역산했다. (Link7 원점 = 팔 원통이 끝나고
그리퍼가 시작되는 플랜지 면. URDF의 4C2_baselink가 z=-0.009인 것으로 확인)

전제:
  터미널1: ros2 launch rm_bringup rm_75_bringup.launch.py
  터미널2: 04의 static TF (Link7 → camera_color_optical_frame)
  터미널3: 이 스크립트

⚠️ 이 스크립트는 MoveIt을 거치지 않는다 = 충돌 검사가 없다.
   작업 공간을 비우고, 웹 UI 비상정지를 띄운 담당자가 있는 상태에서만 실행한다.
"""

import statistics
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO

from rm_ros_interfaces.msg import Movejp
from std_msgs.msg import Bool

# ── 비전 설정 (A단계 v3와 동일) ────────────────────────
TARGET_CLASS = "bottle"
CONF_THRESHOLD = 0.5
DEPTH_MIN, DEPTH_MAX = 0.15, 2.0
SMOOTH_WINDOW = 7
ROI_RATIO = 0.3
MAX_RADIUS = 0.06
CAMERA_FRAME = "camera_color_optical_frame"
BASE_FRAME = "base_link"
MAX_TF_AGE = 0.3
STILL_THRESHOLD = 0.002

# ── 파지 기하 ─────────────────────────────────────────
GRASP_OFFSET = 0.098      # 물병 앞면 → Link7 (실측)
APPROACH_BACK = 0.10      # 파지 위치에서 뒤로 물러날 거리
FIXED_Z = 0.083           # 이번 테스트는 높이를 실측값으로 고정
ORIENTATION = (0.0, 0.707, 0.0, 0.707)   # 측면 파지 (그리퍼가 +x)
SPEED = 10

# ── 안전 한계 (이 범위를 벗어나면 발행하지 않는다) ────
X_RANGE = (0.15, 0.55)
Y_RANGE = (-0.35, 0.35)
Z_RANGE = (-0.15, 0.45)
MAX_STEP = 0.70           # 현재 Link7 위치에서 이보다 멀면 거부 (m). 0.35는 홈→접근 이동에 부족했음


class VisionApproach(Node):
    def __init__(self):
        super().__init__("vision_approach_test")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.movejp_publisher = self.create_publisher(
            Movejp, "/rm_driver/movej_p_cmd", 10
        )
        self.create_subscription(
            Bool, "/rm_driver/movej_p_result", self.on_result, 10
        )
        self.last_result = None

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.intrinsics = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )

        self.model = YOLO("yolov8n.pt")
        self.history = {"x": [], "y": [], "z": []}
        self.previous_camera_position = None
        self.still_since = None

        self.latest = None            # (base_point, radius)
        self.lock = threading.Lock()

    def on_result(self, message):
        self.last_result = message.data

    # ── 현재 Link7 위치 ──
    def current_link7(self):
        transform = self.tf_buffer.lookup_transform(
            BASE_FRAME, "Link7", rclpy.time.Time()
        )
        t = transform.transform.translation
        return np.array([t.x, t.y, t.z])

    # ── 카메라 좌표 + 반지름 ──
    def camera_xyz(self, color_image, depth_frame):
        results = self.model(color_image, verbose=False)[0]

        best = None
        for box in results.boxes:
            name = self.model.names[int(box.cls[0])]
            confidence = float(box.conf[0])
            if name == TARGET_CLASS and confidence >= CONF_THRESHOLD:
                if best is None or confidence > best[1]:
                    best = (box, confidence)

        if best is None:
            self.history = {"x": [], "y": [], "z": []}
            self.previous_camera_position = None
            self.still_since = None
            return None, 0.0, color_image

        box, confidence = best
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        half_w = max(int((x2 - x1) * ROI_RATIO / 2), 2)
        half_h = max(int((y2 - y1) * ROI_RATIO / 2), 2)

        depths = []
        for v in range(cy - half_h, cy + half_h + 1, 2):
            for u in range(cx - half_w, cx + half_w + 1, 2):
                if 0 <= u < 640 and 0 <= v < 480:
                    d = depth_frame.get_distance(u, v)
                    if DEPTH_MIN < d < DEPTH_MAX:
                        depths.append(d)

        if len(depths) < 5:
            return None, 0.0, color_image

        depth = statistics.median(depths)
        point = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [cx, cy], depth
        )

        # 표면점 → 중심축 보정
        width_m = (x2 - x1) * depth / self.intrinsics.fx
        radius = min(width_m / 2.0, MAX_RADIUS)

        direction = np.array(point, dtype=float)
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            point = (direction + radius * direction / norm).tolist()

        for key, value in zip(("x", "y", "z"), point):
            self.history[key].append(value)
            if len(self.history[key]) > SMOOTH_WINDOW:
                self.history[key].pop(0)

        smoothed = np.array([
            statistics.median(self.history[key]) for key in ("x", "y", "z")
        ])

        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(color_image, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(
            color_image,
            f"{TARGET_CLASS} {confidence:.2f} d={depth:.3f} r={radius*1000:.0f}mm",
            (x1, max(y1 - 8, 20)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
        )

        return smoothed, radius, color_image

    def to_base(self, camera_point):
        transform = self.tf_buffer.lookup_transform(
            BASE_FRAME, CAMERA_FRAME, rclpy.time.Time()
        )
        stamp = transform.header.stamp
        age = (
            self.get_clock().now().nanoseconds * 1e-9
            - (stamp.sec + stamp.nanosec * 1e-9)
        )
        t = transform.transform.translation
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        return rotation @ camera_point + np.array([t.x, t.y, t.z]), age

    # ── 영상 루프 ──
    def run(self, stop_event):
        while not stop_event.is_set():
            frames = self.align.process(self.pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            camera_point, radius, color_image = self.camera_xyz(
                color_image, depth_frame
            )

            status, color = "NO DETECTION", (0, 0, 255)

            if camera_point is not None:
                if self.previous_camera_position is not None:
                    delta = np.linalg.norm(
                        camera_point - self.previous_camera_position
                    )
                    if delta < STILL_THRESHOLD:
                        if self.still_since is None:
                            self.still_since = time.time()
                    else:
                        self.still_since = None
                self.previous_camera_position = camera_point.copy()

                try:
                    base_point, age = self.to_base(camera_point)
                except Exception:
                    base_point, age = None, None

                if base_point is None:
                    status, color = "TF WAIT", (0, 165, 255)
                elif age > MAX_TF_AGE:
                    status, color = f"TF STALE {age*1000:.0f}ms", (0, 0, 255)
                elif (
                    self.still_since is None
                    or time.time() - self.still_since < 0.7
                ):
                    status, color = "MOVING", (0, 165, 255)
                else:
                    status, color = "READY", (0, 255, 0)
                    with self.lock:
                        self.latest = (base_point, radius)

                if base_point is not None:
                    cv2.putText(
                        color_image,
                        f"bottle X={base_point[0]:+.4f} "
                        f"Y={base_point[1]:+.4f} Z={base_point[2]:+.4f}",
                        (10, 460), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 0), 2,
                    )

            if status != "READY":
                with self.lock:
                    self.latest = None

            cv2.putText(
                color_image, status, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2,
            )
            cv2.imshow("vision_approach_test (B)", color_image)
            cv2.waitKey(1)

        self.pipeline.stop()
        cv2.destroyAllWindows()

    # ── 접근 명령 ──
    def approach(self):
        with self.lock:
            latest = self.latest

        if latest is None:
            print("  READY 상태가 아닙니다.")
            return

        bottle, radius = latest

        front_x = bottle[0] - radius
        grasp_x = front_x - GRASP_OFFSET
        approach_x = grasp_x - APPROACH_BACK

        target = np.array([approach_x, bottle[1], FIXED_Z])

        print(f"\n  물병 중심   X={bottle[0]:+.4f} Y={bottle[1]:+.4f} "
              f"Z={bottle[2]:+.4f}  (반지름 {radius*1000:.0f}mm)")
        print(f"  물병 앞면   X={front_x:+.4f}")
        print(f"  파지 위치   X={grasp_x:+.4f}  (이번엔 여기까지 가지 않음)")
        print(f"  접근 목표   X={target[0]:+.4f} Y={target[1]:+.4f} "
              f"Z={target[2]:+.4f}")

        # 안전 점검
        problems = []
        for name, value, (low, high) in (
            ("x", target[0], X_RANGE),
            ("y", target[1], Y_RANGE),
            ("z", target[2], Z_RANGE),
        ):
            if not (low <= value <= high):
                problems.append(
                    f"{name}={value:+.4f} 가 허용 범위 {low}~{high} 밖"
                )

        try:
            current = self.current_link7()
            step = np.linalg.norm(target - current)
            print(f"  현재 Link7  X={current[0]:+.4f} Y={current[1]:+.4f} "
                  f"Z={current[2]:+.4f}   → 이동량 {step*1000:.0f}mm")
            if step > MAX_STEP:
                problems.append(
                    f"이동량 {step*1000:.0f}mm 가 한계 {MAX_STEP*1000:.0f}mm 초과"
                )
        except Exception as error:
            problems.append(f"현재 위치를 읽지 못함: {error}")

        if problems:
            print("\n  [거부] 발행하지 않습니다:")
            for problem in problems:
                print(f"    - {problem}")
            return

        answer = input("\n  발행하려면 'go' 입력 (그 외는 취소): ")
        if answer.strip().lower() != "go":
            print("  취소했습니다.")
            return

        message = Movejp()
        message.pose.position.x = float(target[0])
        message.pose.position.y = float(target[1])
        message.pose.position.z = float(target[2])
        message.pose.orientation.x = ORIENTATION[0]
        message.pose.orientation.y = ORIENTATION[1]
        message.pose.orientation.z = ORIENTATION[2]
        message.pose.orientation.w = ORIENTATION[3]
        message.speed = SPEED
        message.trajectory_connect = 0
        message.block = True

        self.last_result = None
        self.movejp_publisher.publish(message)
        print("  발행했습니다. 결과 대기 중...")

        for _ in range(200):          # 최대 20초
            if self.last_result is not None:
                break
            time.sleep(0.1)

        if self.last_result is None:
            print("  결과 회신이 없습니다 (타임아웃).")
        elif self.last_result:
            print("  이동 성공. 그리퍼와 물병의 정렬을 눈으로 확인하세요.")
        else:
            print("  이동 실패 (도달 불가 가능성).")


def main():
    rclpy.init()
    node = VisionApproach()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    stop_event = threading.Event()
    vision_thread = threading.Thread(
        target=node.run, args=(stop_event,), daemon=True
    )
    vision_thread.start()

    time.sleep(2.0)

    print("\n" + "=" * 60)
    print("  [B단계] 비전 좌표로 접근 — 파지하지 않습니다")
    print("  작업 공간을 비우고, 비상정지 담당자를 확인하세요.")
    print("=" * 60)
    print("\n영상 창이 READY(초록)일 때 Enter. 종료: q + Enter\n")

    try:
        while True:
            key = input("Enter=접근 계산, q=종료: ")
            if key.strip().lower() == "q":
                break
            node.approach()
    except (KeyboardInterrupt, EOFError):
        pass

    stop_event.set()
    vision_thread.join(timeout=2.0)
    executor.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
