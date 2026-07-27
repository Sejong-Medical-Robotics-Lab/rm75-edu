#!/usr/bin/env python3
"""예제 — 미션 Scene 구성 (교재 4.6 체크포인트 ⑦ · 7.2 설계 회의의 '15분').

Planning Scene 에 테이블·블록 A·B 표식을 등록한다 — 등록된 것만 플래너가
피한다("Scene 에 없는 실물은 보이지 않는다", 교재 4.5).

    ros2 launch rm75_edu_moveit_config demo.launch.py   # 터미널 1
    ros2 run rm75_edu mission_scene                     # 터미널 2

좌표는 rm75_edu/poses.py 와 동기 — 바꾸면 pick_place_template 도 함께 본다.
"""
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from rm75_edu.poses import A_XY, B_XY, BLOCK_SIZE, TABLE_TOP_Z


def box(node, name, size, xyz):
    co = CollisionObject()
    co.header.frame_id = "base_link"
    co.id = name
    p = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    pose.orientation.w = 1.0
    co.primitives.append(p)
    co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


def main():
    rclpy.init()
    node = rclpy.create_node("mission_scene")
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=5.0):
        raise SystemExit("/apply_planning_scene 서비스가 없습니다 — demo 런치 먼저.")

    th = 0.04                                   # 테이블 두께
    objs = [
        # 테이블: 윗면이 TABLE_TOP_Z (로봇 베이스가 그 위에 서 있는 그림)
        box(node, "table", (1.0, 0.9, th),
            (0.35, 0.0, TABLE_TOP_Z - th / 2)),
        # 블록 A(집을 물체): 테이블 위 정육면체
        box(node, "block_a", (BLOCK_SIZE,) * 3,
            (A_XY[0], A_XY[1], TABLE_TOP_Z + BLOCK_SIZE / 2)),
        # B 표식: 내려놓을 자리(얇은 판 — 테이프의 역할)
        box(node, "target_b", (0.10, 0.10, 0.002),
            (B_XY[0], B_XY[1], TABLE_TOP_Z + 0.001)),
    ]
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.extend(objs)

    req = ApplyPlanningScene.Request()
    req.scene = scene
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    ok = fut.result() and fut.result().success
    print("Scene 등록:", "성공 — RViz 에서 테이블·블록 A·B 표식 확인"
          if ok else "실패")
    print("이제 블록 A 근처로 Plan 하면 경로가 테이블·블록을 피해 계획됩니다"
          " (체크포인트 ⑥의 미션판).")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
