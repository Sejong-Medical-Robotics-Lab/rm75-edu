"""
RealSense 컬러 영상에서 체커보드를 검출해
camera_color_optical_frame -> checkerboard TF를 발행한다.
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import yaml

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

CAMERA_FRAME = "camera_color_optical_frame"
BOARD_FRAME = "checkerboard"

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

PUBLISH_HZ = 10.0

SHOW_WINDOW = True

CORNER_CRITERIA = (
    cv2.TERM_CRITERIA_EPS
    + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)

class CheckerboardTfPublisher(Node):

    def __init__(self):
        super().__init__("checkerboard_tf_publisher")

        # 캘리브레이션과 동일한 체커보드 설정을 사용한다.
        with open(
            "config.yaml",
            "r",
            encoding="utf-8",
        ) as file:
            config = yaml.safe_load(file)

        args = config.get("checkerboard_args")

        self.xx = args.get("XX")
        self.yy = args.get("YY")
        self.square = args.get("L")

        # compute_in_hand.py와 동일한 방식으로 3D 점 생성
        objp = np.zeros(
            (self.xx * self.yy, 3),
            np.float32,
        )

        objp[:, :2] = np.mgrid[
            0:self.xx,
            0:self.yy,
        ].T.reshape(-1, 2)

        self.objp = self.square * objp

        self.broadcaster = TransformBroadcaster(self)

        self.pipeline = rs.pipeline()
        rs_config = rs.config()

        rs_config.enable_stream(
            rs.stream.color,
            CAMERA_WIDTH,
            CAMERA_HEIGHT,
            rs.format.bgr8,
            CAMERA_FPS,
        )

        profile = self.pipeline.start(rs_config)

        # RealSense 공장 내참 사용
        color_stream = profile.get_stream(
            rs.stream.color
        ).as_video_stream_profile()

        intrinsics = color_stream.get_intrinsics()

        self.camera_matrix = np.array(
            [
                [intrinsics.fx, 0.0, intrinsics.ppx],
                [0.0, intrinsics.fy, intrinsics.ppy],
                [0.0, 0.0, 1.0],
            ]
        )

        self.dist_coeffs = np.array(
            intrinsics.coeffs,
            dtype=np.float64,
        )

        self.get_logger().info(
            f"체커보드 내부 코너: {self.xx} x {self.yy}, "
            f"한 칸: {self.square} m"
        )

        self.get_logger().info(
            f"fx={intrinsics.fx:.2f} "
            f"fy={intrinsics.fy:.2f} "
            f"cx={intrinsics.ppx:.2f} "
            f"cy={intrinsics.ppy:.2f}"
        )

        self.timer = self.create_timer(
            1.0 / PUBLISH_HZ,
            self.process_frame,
        )

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()

        if not color_frame:
            return

        image = np.asanyarray(color_frame.get_data())

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY,
        )

        found, corners = cv2.findChessboardCorners(
            gray,
            (self.xx, self.yy),
            None,
        )

        if found:
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (5, 5),
                (-1, -1),
                CORNER_CRITERIA,
            )

            success, rvec, tvec = cv2.solvePnP(
                self.objp,
                corners,
                self.camera_matrix,
                self.dist_coeffs,
            )

            if success:
                self.publish_tf(rvec, tvec)

            if SHOW_WINDOW:
                cv2.drawChessboardCorners(
                    image,
                    (self.xx, self.yy),
                    corners,
                    found,
                )

        if SHOW_WINDOW:
            status = "DETECTED" if found else "NOT DETECTED"
            color = (0, 255, 0) if found else (0, 0, 255)

            cv2.putText(
                image,
                status,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                color,
                2,
            )

            cv2.imshow("Checkerboard TF", image)
            cv2.waitKey(1)

    def publish_tf(self, rvec, tvec):
        rotation = Rotation.from_rotvec(
            rvec.reshape(3)
        )

        quaternion = rotation.as_quat()  # x, y, z, w

        transform = TransformStamped()

        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )

        transform.header.frame_id = CAMERA_FRAME
        transform.child_frame_id = BOARD_FRAME

        transform.transform.translation.x = float(tvec[0])
        transform.transform.translation.y = float(tvec[1])
        transform.transform.translation.z = float(tvec[2])

        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])

        self.broadcaster.sendTransform(transform)

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()

    node = CheckerboardTfPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
