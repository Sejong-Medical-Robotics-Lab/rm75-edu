"""
[05 테스트 C] 비전 파지 — 접근부터 들어올리기까지

B단계의 접근에 이어, 03에서 검증된 파지 시퀀스를 비전 좌표로 수행한다.

    [1] MoveJ_P  접근 위치로            (파지 x - 0.10)
    [2] 그리퍼 최대 개방
    [3] MoveL    파지 위치로 직진        (물병이 손가락 사이로)
    [4] pick_on  힘 제어 파지
    [5] MoveL    5cm 들어올리기
    --- 확인 후 ---
    [6] MoveL    내려놓기 → 개방 → 후퇴

핵심 설계: 'go' 입력 시점의 비전 좌표를 고정(latch)하고, 이후 모든 단계는
그 값만 쓴다. 팔이 움직이면 시야가 바뀌어 검출이 흔들리므로 중간 재검출은
하지 않는다.

전제:
  터미널1: ros2 launch rm_bringup rm_75_bringup.launch.py
  터미널2: 04의 static TF
  터미널3: 이 스크립트

⚠️ MoveIt을 거치지 않는다 = 충돌 검사 없음.
   각 단계마다 확인을 요구하며, 아무 때나 q로 중단할 수 있다.
   작업 공간을 비우고 웹 UI 비상정지 담당자가 있는 상태에서만 실행한다.
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

from rm_ros_interfaces.msg import Gripperpick, Gripperset, Movejp, Movel
from std_msgs.msg import Bool

# ── 비전 설정 (B와 동일) ─────────────────────────────
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

# ── 파지 기하 ────────────────────────────────────────
GRASP_OFFSET = 0.098
APPROACH_BACK = 0.10
FIXED_Z = 0.083
LIFT_HEIGHT = 0.05
ORIENTATION = (0.0, 0.707, 0.0, 0.707)
SPEED = 10

# ── 그리퍼 ───────────────────────────────────────────
GRIP_SPEED = 200
GRIP_FORCE = 600     # 400에서는 상승 중 미끄러짐 발생 → 600으로 확정 (물 찬 병 기준)
OPEN_POSITION = 1000

# ── 안전 한계 ────────────────────────────────────────
X_RANGE = (0.15, 0.55)
Y_RANGE = (-0.35, 0.35)
Z_RANGE = (-0.15, 0.45)
MAX_STEP = 0.70


class VisionGrasp(Node):
    def __init__(self):
        super().__init__("vision_grasp_test")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_movejp = self.create_publisher(
            Movejp, "/rm_driver/movej_p_cmd", 10
        )
        self.pub_movel = self.create_publisher(
            Movel, "/rm_driver/movel_cmd", 10
        )
        self.pub_grip_set = self.create_publisher(
            Gripperset, "/rm_driver/set_gripper_position_cmd", 10
        )
        self.pub_grip_pick = self.create_publisher(
            Gripperpick, "/rm_driver/set_gripper_pick_on_cmd", 10
        )

        self.results = {}
        for key, topic in (
            ("movejp", "/rm_driver/movej_p_result"),
            ("movel", "/rm_driver/movel_result"),
            ("grip_set", "/rm_driver/set_gripper_position_result"),
            ("grip_pick", "/rm_driver/set_gripper_pick_on_result"),
        ):
            self.create_subscription(
                Bool, topic,
                lambda message, k=key: self.results.__setitem__(
                    k, message.data
                ),
                10,
            )

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
        self.latest = None
        self.lock = threading.Lock()

    # ── 결과 대기 ──
    def wait_result(self, key, timeout=25.0):
        self.results.pop(key, None)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if key in self.results:
                return self.results[key]
            time.sleep(0.1)
        return None

    def current_link7(self):
        transform = self.tf_buffer.lookup_transform(
            BASE_FRAME, "Link7", rclpy.time.Time()
        )
        t = transform.transform.translation
        return np.array([t.x, t.y, t.z])

    # ── 비전 (B와 동일) ──
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
        cv2.putText(
            color_image,
            f"{TARGET_CLASS} {confidence:.2f} r={radius*1000:.0f}mm",
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
                    status, color = "TF STALE", (0, 0, 255)
                elif (
                    self.still_since is None
                    or time.time() - self.still_since < 0.7
                ):
                    status, color = "MOVING", (0, 165, 255)
                else:
                    status, color = "READY", (0, 255, 0)
                    with self.lock:
                        self.latest = (base_point, radius)

            if status != "READY":
                with self.lock:
                    self.latest = None

            cv2.putText(
                color_image, status, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2,
            )
            cv2.imshow("vision_grasp_test (C)", color_image)
            cv2.waitKey(1)

        self.pipeline.stop()
        cv2.destroyAllWindows()

    # ── 이동/그리퍼 프리미티브 ──
    def check_target(self, target):
        problems = []
        for name, value, (low, high) in (
            ("x", target[0], X_RANGE),
            ("y", target[1], Y_RANGE),
            ("z", target[2], Z_RANGE),
        ):
            if not (low <= value <= high):
                problems.append(f"{name}={value:+.4f} 범위 밖 ({low}~{high})")
        try:
            step = np.linalg.norm(target - self.current_link7())
            if step > MAX_STEP:
                problems.append(f"이동량 {step*1000:.0f}mm 초과")
        except Exception as error:
            problems.append(f"현재 위치 읽기 실패: {error}")
        return problems

    def move(self, kind, target):
        problems = self.check_target(target)
        if problems:
            for problem in problems:
                print(f"    [거부] {problem}")
            return False

        if kind == "movejp":
            message = Movejp()
        else:
            message = Movel()
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

        (self.pub_movejp if kind == "movejp" else self.pub_movel).publish(
            message
        )
        result = self.wait_result(kind)
        print(f"    결과: {result}")
        return result is True

    def gripper_open(self):
        message = Gripperset()
        message.position = OPEN_POSITION
        message.block = True
        message.timeout = 10
        self.pub_grip_set.publish(message)
        result = self.wait_result("grip_set")
        print(f"    결과: {result}  ← 실제로 열렸는지 눈으로 확인")
        return result is True

    def gripper_pick(self):
        message = Gripperpick()
        message.speed = GRIP_SPEED
        message.force = GRIP_FORCE
        message.block = True
        message.timeout = 10
        self.pub_grip_pick.publish(message)
        result = self.wait_result("grip_pick")
        print(f"    결과: {result}  ← 물병이 물렸는지 눈으로 확인")
        return result is True

    # ── 시퀀스 ──
    def sequence(self):
        with self.lock:
            latest = self.latest
        if latest is None:
            print("  READY 상태가 아닙니다.")
            return

        bottle, radius = latest              # ★ 여기서 고정 (latch)
        front_x = bottle[0] - radius
        grasp = np.array([front_x - GRASP_OFFSET, bottle[1], FIXED_Z])
        approach = grasp - np.array([APPROACH_BACK, 0.0, 0.0])
        lift = grasp + np.array([0.0, 0.0, LIFT_HEIGHT])

        print(f"\n  물병 중심 X={bottle[0]:+.4f} Y={bottle[1]:+.4f} "
              f"(반지름 {radius*1000:.0f}mm)")
        print(f"  접근 {approach.round(4)}")
        print(f"  파지 {grasp.round(4)}")
        print(f"  상승 {lift.round(4)}")
        print("  ※ 이 값으로 고정합니다. 이후 물병을 움직이지 마세요.\n")

        steps = [
            ("1) MoveJ_P 접근", lambda: self.move("movejp", approach)),
            ("2) 그리퍼 개방", self.gripper_open),
            ("3) MoveL 파지 위치로 직진", lambda: self.move("movel", grasp)),
            ("4) pick_on 파지", self.gripper_pick),
            ("5) MoveL 5cm 상승", lambda: self.move("movel", lift)),
        ]

        for name, action in steps:
            answer = input(f"  {name} — 실행하려면 Enter, 중단은 q: ")
            if answer.strip().lower() == "q":
                print("  중단했습니다.")
                return
            if not action():
                print("  단계 실패 — 시퀀스를 멈춥니다.")
                return

        print("\n  ✓ 파지 완료. 물병이 들려 있는지 확인하세요.")
        answer = input(
            "  6) 내려놓기(하강→개방→후퇴)를 실행하려면 Enter, 유지는 q: "
        )
        if answer.strip().lower() == "q":
            return

        if not self.move("movel", grasp):
            return
        if not self.gripper_open():
            return
        print("    물병이 안정된 것을 확인한 뒤 후퇴합니다.")
        input("    Enter를 누르면 후퇴: ")
        self.move("movel", approach)
        print("  ✓ 시퀀스 종료.")


def main():
    rclpy.init()
    node = VisionGrasp()

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
    print("  [C단계] 비전 파지 — 단계별 확인 방식")
    print("  작업 공간 비우기 · 웹 UI 비상정지 담당자 확인")
    print("=" * 60)
    print("\n영상 창이 READY(초록)일 때 Enter. 종료: q + Enter\n")

    try:
        while True:
            key = input("Enter=시퀀스 시작, q=종료: ")
            if key.strip().lower() == "q":
                break
            node.sequence()
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
