"""
RM75의 base_link -> Link7 변환을 TF로 실시간 발행한다.
Robotic_Arm SDK를 사용하므로 rm_driver 없이 동작한다.
"""

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

from Robotic_Arm.rm_robot_interface import (
    RoboticArm,
    rm_thread_mode_e,
)

ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080

BASE_FRAME = "base_link"
TOOL_FRAME = "Link7"

PUBLISH_HZ = 20.0

class RmTfPublisher(Node):

    def __init__(self):
        super().__init__("rm_tf_publisher")

        self.broadcaster = TransformBroadcaster(self)

        self.arm = RoboticArm(
            rm_thread_mode_e.RM_TRIPLE_MODE_E
        )

        handle = self.arm.rm_create_robot_arm(
            ROBOT_IP,
            ROBOT_PORT,
        )

        if handle.id < 0:
            raise RuntimeError("로봇 연결 실패")

        self.get_logger().info(
            f"로봇 연결 성공: {ROBOT_IP}"
        )

        self.timer = self.create_timer(
            1.0 / PUBLISH_HZ,
            self.publish_tf,
        )

    def publish_tf(self):
        code, state = self.arm.rm_get_current_arm_state()

        if code != 0:
            self.get_logger().warn(
                f"상태 조회 실패: {code}"
            )
            return

        pose = state["pose"]

        # pose = [x, y, z, rx, ry, rz]
        # 위치 단위 m, 자세 단위 rad (RPY)
        rotation = Rotation.from_euler(
            "xyz",
            [pose[3], pose[4], pose[5]],
        )

        quaternion = rotation.as_quat()  # x, y, z, w

        transform = TransformStamped()

        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )

        transform.header.frame_id = BASE_FRAME
        transform.child_frame_id = TOOL_FRAME

        transform.transform.translation.x = float(pose[0])
        transform.transform.translation.y = float(pose[1])
        transform.transform.translation.z = float(pose[2])

        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])

        self.broadcaster.sendTransform(transform)

    def destroy_node(self):
        self.arm.rm_delete_robot_arm()
        super().destroy_node()

def main():
    rclpy.init()

    node = RmTfPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()-
