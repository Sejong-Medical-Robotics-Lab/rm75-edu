#!/usr/bin/env python3
"""예제 — 그리퍼 세 숫자 놀이 (교재 5.3).

    ros2 launch rm75_edu_moveit_config demo.launch.py   # 터미널 1
    ros2 run rm75_edu gripper_demo                      # 터미널 2

RViz 에서 손가락이 열리고 닫히는 것을 관찰하며, /joint_states 의
gripper_finger* 값이 함께 변하는 것을 echo 로 확인해 보라(미션 A ②의 감각).
"""
import time

import rclpy

from rm75_edu.gripper_client import GripperClient
from rm75_edu.poses import BLOCK_SIZE


def main():
    rclpy.init()
    g = GripperClient()
    try:
        print("열기 (폭 0.08)");           g.open();  time.sleep(0.5)
        print("닫기 (폭 0.00)");           g.close(); time.sleep(0.5)
        print("파지 — 블록보다 약간 작게 (교재 5.3: '물체보다 약간 작게')")
        g.grasp(width=BLOCK_SIZE - 0.005, speed=0.02, force=5.0)
        time.sleep(0.5)
        print("다시 열기");                 g.open()
    finally:
        g.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
