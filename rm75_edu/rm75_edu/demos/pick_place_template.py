#!/usr/bin/env python3
"""예제 — pick-and-place 시퀀스 템플릿 (교재 7.2 '시퀀스 표'를 코드로).

설계 시트의 표를 SEQUENCE 리스트에 한 줄씩 옮긴다. 지금은 출발·①구간만
채워져 있다 — 나머지는 여러분의 설계다(7.2 설계 회의 → 7.3 단계별 통과).

    ros2 launch rm75_edu_moveit_config demo.launch.py   # 터미널 1
    ros2 run rm75_edu mission_scene                     # 터미널 2 (Scene 먼저!)
    ros2 run rm75_edu pick_place_template               # 터미널 3

행 형식:
    ("named",  "home", None)                    # 이름 붙은 자세로
    ("pose",   (x, y, z), None)                 # TCP 목표 (아래보기 상수 사용)
    ("gripper", "open"|"close", None)           # 열기/닫기
    ("grasp",  (width, speed, force), None)     # 파지 — 성공 확인 포함
각 팔 이동은 plan → 성공 확인 → execute 로 실행된다(교재 5.1의 원칙이
ArmClient 안에 강제되어 있다).
"""
import rclpy

from rm75_edu.arm_client import ArmClient
from rm75_edu.gripper_client import GripperClient
from rm75_edu.poses import (A_XY, APPROACH_Z, B_XY, BLOCK_SIZE, GRASP_Z,
                            PLACE_Z)

# ── 여기를 채우세요 (교재 7.2 시퀀스 표 — 접근·하강·파지·이동·안착·복귀) ──
SEQUENCE = [
    ("named",   "home",                          "출발: 홈"),
    ("gripper", "open",                          "그리퍼 열기"),
    ("pose",    (A_XY[0], A_XY[1], APPROACH_Z),  "① A 위 접근 자세"),
    # TODO ②: 파지 자세로 수직 하강      ("pose", (A_XY[0], A_XY[1], GRASP_Z), ...)
    # TODO ③: 그리퍼 파지               ("grasp", (BLOCK_SIZE-0.005, 0.02, 5.0), ...)
    # TODO ④: 들어올림 → B 위로 이동
    # TODO ⑤: 하강 → 열기 → 상승
    ("named",   "home",                          "복귀: 홈"),
]
# ────────────────────────────────────────────────────────────────────


def main():
    rclpy.init()
    node = rclpy.create_node("pick_place")
    arm = ArmClient(node)
    grip = GripperClient(node)
    try:
        for i, (kind, arg, note) in enumerate(SEQUENCE, 1):
            print(f"[{i}/{len(SEQUENCE)}] {note or kind} {arg}")
            if kind == "named":
                ok = arm.go_named(arg)
            elif kind == "pose":
                ok = arm.go_pose(*arg)          # plan→확인→execute 내장
            elif kind == "gripper":
                ok = grip.open() if arg == "open" else grip.close()
            elif kind == "grasp":
                ok = grip.grasp(*arg)
            else:
                ok = False
            if not ok:
                print("→ 이 구간 실패 — 시퀀스를 중단합니다 "
                      "(실패 도감 7.5로 원인 분류 후 값 하나만 수정).")
                break
        else:
            print("시퀀스 완주 — demo 검증 완료. 다음은 7.3 단계별 실기체.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
