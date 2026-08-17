"""
[06 테스트 F] 가상 장애물 등록/제거 — 충돌 회피를 눈으로 실증하기

Scene에 세로 패널(장애물)을 하나 세워 두고 E 테스트를 다시 돌리면,
같은 물병·같은 목표인데 접근 경로가 패널을 피해 돌아가는 것을
RViz 잔상과 실물 팔 양쪽에서 볼 수 있다. 이것이 05(기본 모션)와
06(MoveIt)의 차이를 보여주는 결정적 실험이다.

가상 장애물이므로 실물이 없어도 안전하다 — 팔이 빈 공중을 돌아간다.
(실물 상자를 같은 위치에 두면 더 실감나지만 필수는 아니다)

사용법:
    python3 add_obstacle.py add                  # 기본 위치, 테이블 위에 세움
    python3 add_obstacle.py remove               # 패널 제거
    python3 add_obstacle.py add 0.30 0.15        # x, y 지정 (테이블 위)
    python3 add_obstacle.py add 0.30 0.15 0.25   # x, y, z 지정 — z는 "패널 바닥"의
                                                 # 높이 (base_link 기준). 공중 장애물
                                                 # (선반 등)을 흉내 낼 때 사용

실행 환경: ROS만 (venv 불필요)
    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
"""

import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

# 패널 기본값 — 물병(x≈0.55) 앞 공간을 부분적으로 막는 위치
DEFAULT_X = 0.32
DEFAULT_Y = 0.12
TABLE_Z = -0.010
PANEL_SIZE = (0.02, 0.20, 0.25)      # 얇은 세로 패널 (x두께, y폭, z높이)
OBSTACLE_ID = "obstacle_panel"


def make_pose(x, y, z):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.w = 1.0
    return p


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "add"
    x = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_X
    y = float(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_Y
    # z 인자 = 패널 "바닥"의 높이. 생략하면 테이블 면 → 테이블 위에 서 있는 물체
    bottom_z = float(sys.argv[4]) if len(sys.argv) > 4 else TABLE_Z

    rclpy.init()
    node = Node("obstacle_tool")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    if not client.wait_for_service(timeout_sec=5.0):
        print("'/apply_planning_scene' 서비스 없음 — bringup이 떠 있나요?")
        rclpy.shutdown()
        return

    obstacle = CollisionObject()
    obstacle.header.frame_id = "base_link"
    obstacle.id = OBSTACLE_ID

    if mode == "remove":
        obstacle.operation = CollisionObject.REMOVE
    else:
        obstacle.primitives = [
            SolidPrimitive(
                type=SolidPrimitive.BOX, dimensions=list(PANEL_SIZE)
            )
        ]
        # 중심 = 바닥 + 높이/2 (박스 원점은 중심이므로)
        obstacle.primitive_poses = [
            make_pose(x, y, bottom_z + PANEL_SIZE[2] / 2.0)
        ]
        obstacle.operation = CollisionObject.ADD

    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects.append(obstacle)

    future = client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()

    ok = result is not None and result.success
    if mode == "remove":
        print(f"패널 제거 {'성공' if ok else '실패'}")
    else:
        print(
            f"패널 등록 {'성공' if ok else '실패'} — "
            f"위치 ({x:.2f}, {y:.2f}), 바닥 z={bottom_z:.3f}, 크기 {PANEL_SIZE}"
        )
        print("RViz에서 패널을 확인하세요 (클릭 금지).")
        print("E 스크립트를 실행하면 접근 계획이 이 패널을 피해 돕니다.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
