"""
RM75의 관절각을 SDK로 읽어 /joint_states로 발행한다.
robot_state_publisher가 이를 받아 URDF 기반 TF를 만든다.
rm_tf_publisher.py와 동시에 실행하면 안 된다 (TF 충돌).
"""

import math

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState

from Robotic_Arm.rm_robot_interface import (
    RoboticArm,
    rm_thread_mode_e,
)

ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080

# URDF의 관절 이름과 반드시 일치해야 한다.
JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
]

PUBLISH_HZ = 20.0

class RmJointPublisher(Node):

    def __init__(self):
        super().__init__("rm_joint_publisher")

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

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
            self.publish_joints,
        )
        self.log_counter = 0

    def publish_joints(self):
        code, state = self.arm.rm_get_current_arm_state()

        if code != 0:
            self.get_logger().warn(
                f"상태 조회 실패: {code}"
            )
            return

        # SDK 관절각은 deg 단위이므로 rad로 변환
        joints_rad = [
            math.radians(angle)
            for angle in state["joint"]
        ]

        message = JointState()

        message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        message.name = JOINT_NAMES
        message.position = joints_rad

        self.publisher.publish(message)

        self.log_counter += 1

        # 1초에 한 번만 출력
        if self.log_counter % 20 == 0:
            pose = state["pose"]

            self.get_logger().info(
                f"SDK pose: "
                f"{pose[0]:+.4f}, "
                f"{pose[1]:+.4f}, "
                f"{pose[2]:+.4f} m | "
                f"RPY "
                f"{pose[3]:+.3f}, "
                f"{pose[4]:+.3f}, "
                f"{pose[5]:+.3f} rad"
            )

    def destroy_node(self):
        self.arm.rm_delete_robot_arm()
        super().destroy_node()

def main():
    rclpy.init()

    node = RmJointPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
