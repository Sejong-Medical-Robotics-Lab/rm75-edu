#!/usr/bin/env python3
"""예제 — "홈 → 목표 → 홈" (교재 5.4 뼈대 · 5.6 미션 B의 그 스크립트).

    # 터미널 1: demo 모드
    ros2 launch rm75_edu_moveit_config demo.launch.py
    # 터미널 2:
    ros2 run rm75_edu move_home_target
"""
import argparse

import rclpy

from rm75_edu.arm_client import ArmClient
from rm75_edu.poses import DOWN_Q


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--x", type=float, default=0.32)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=0.25)
    args, _ = ap.parse_known_args(argv)

    # ① 초기화 — MoveIt 인터페이스·그룹 준비 (로봇과 대화할 채비)
    rclpy.init()
    arm = ArmClient()

    try:
        # ② 홈 복귀 — named goal 로 알려진 자세에서 출발
        ok, traj, msg = arm.plan_named("home")
        print("홈:", msg)
        if not ok or not arm.execute(traj):
            print("홈 복귀 실패 — 종료합니다")
            return

        # ③ 목표 설정 — pose goal (position + 재사용하는 '아래 보기' 상수)
        print(f"목표: ({args.x}, {args.y}, {args.z}) + DOWN_Q {DOWN_Q}")
        # ④ plan() — 궤적 계산
        ok, traj, msg = arm.plan_pose(args.x, args.y, args.z, DOWN_Q)
        print("목표:", msg)
        # ⑤ 성공 확인 — if 실패 → 실행하지 않고 이유 출력 후 종료
        if not ok:
            print("→ 실패한 계획은 실행하지 않습니다 (교재 5.1). "
                  "부록 B의 세 갈래(도달·자세·충돌)로 원인을 좁혀 보세요.")
            return
        # ⑥ execute() — 검증된 궤적만 실행
        if not arm.execute(traj):
            print("실행 실패")
            return
        print("도달 — RViz 에서 손끝(TCP)이 목표를 가리키는지 눈으로 대조 (5.5)")

        # ⑦ 홈 복귀 — "끝도 알려진 상태로"
        arm.go_named("home")
        print("완료: 홈 복귀")
    finally:
        arm.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
