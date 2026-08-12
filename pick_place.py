#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RM75 + Inspire EG2-4C2 픽앤플레이스 (시뮬레이션 검증용).

물체 좌표를 주면 접근 → 파지 → 이송 → 놓기까지 스스로 수행한다.

준비:
    ros2 launch rm_75_jaw_config demo.launch.py     # 터미널 1
    python3 pick_place.py                           # 터미널 2

의존 패키지는 moveit_msgs / shape_msgs / geometry_msgs 뿐이라 별도 설치 없이 돌아간다.

이 스크립트에 이미 반영된 두 가지 순서 규칙 (바꾸면 실패한다):
    * pick  : 그리퍼를 닫기 "전"에 attach — touch_links가 "손가락이 이 물체에
              닿아도 됨"이라는 허가증이므로, 접촉이 생기기 전에 발급해야 한다.
    * place : attach 상태에서 그리퍼를 열고 → 살짝 후퇴해 손가락이 물체에서
              완전히 빠진 "뒤"에 detach — 순서를 바꾸면 그리퍼 열기가
              충돌 판정으로 실패한다.
"""
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from shape_msgs.msg import SolidPrimitive

# ─────────────────────────────────────────────────────────────
# 설정 — 환경에 맞게 여기만 고치면 된다
# ─────────────────────────────────────────────────────────────
BASE_FRAME = "base_link"
ARM_GROUP = "rm_group"
GRIPPER_GROUP = "gripper"
TCP_LINK = "grasp_tcp"          # URDF에 추가한 파지 기준점
ATTACH_LINK = "Link7"           # 물체를 붙일 링크

GRIPPER_JOINT = "jaw_Joint1"
GRIPPER_OPEN = 0.80             # rad (한계 0.82보다 살짝 안쪽)
GRIPPER_CLOSED = 0.10           # 완전히 0으로 닫으면 물체를 뚫는다

# 그리퍼를 바닥으로 향하게 하는 자세 (x, y, z, w)
# x축 180° 회전 → 공구 +z 가 월드 -z
DOWN_Q = (1.0, 0.0, 0.0, 0.0)

APPROACH_HEIGHT = 0.12          # 물체 위 몇 m 에서 접근 시작할지
RETREAT_LIFT = 0.06             # place 후 detach 전 이탈 높이
VEL_SCALE = 0.1
ACC_SCALE = 0.1

# 손가락이 물체를 감쌀 여유를 고려한 파지 링크 목록
# (attach 시 이 링크들과의 충돌은 무시된다)
TOUCH_LINKS = [
    "Link7", "4C2_baselink",
    "4C2_Link1", "4C2_Link2", "4C2_Link3",
    "4C2_Link4", "4C2_Link5", "4C2_Link6",
    "grasp_tcp",
]


def make_pose(x, y, z, quat=DOWN_Q):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = \
        (float(v) for v in quat)
    return p


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place")

        self.move_cli = ActionClient(self, MoveGroup, "/move_action")
        self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")

        for name, cli in (("/move_action", self.move_cli),
                          ("/execute_trajectory", self.exec_cli)):
            if not cli.wait_for_server(timeout_sec=10.0):
                raise SystemExit(f"{name} 액션 서버 없음 — demo.launch.py 를 먼저 띄우세요.")

        for name, cli in (("/apply_planning_scene", self.scene_cli),
                          ("/compute_cartesian_path", self.cart_cli)):
            if not cli.wait_for_service(timeout_sec=10.0):
                raise SystemExit(f"{name} 서비스 없음 — demo.launch.py 를 먼저 띄우세요.")

        self.get_logger().info("MoveIt 연결 완료")

    # ── 내부 유틸 ────────────────────────────────────────────
    def _call(self, client, request):
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def _send_goal(self, client, goal):
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return None
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return res_fut.result().result

    # ── Planning Scene ──────────────────────────────────────
    def add_box(self, name, size, xyz):
        """씬에 박스를 등록한다. size/xyz 는 (x, y, z) 튜플."""
        co = CollisionObject()
        co.header.frame_id = BASE_FRAME
        co.id = name
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                        dimensions=[float(v) for v in size])]
        co.primitive_poses = [make_pose(*xyz, quat=(0.0, 0.0, 0.0, 1.0))]
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)

        req = ApplyPlanningScene.Request(scene=scene)
        ok = self._call(self.scene_cli, req)
        self.get_logger().info(f"[scene] {name} 등록 {'성공' if ok else '실패'}")
        return bool(ok)

    def attach(self, name):
        """물체를 그리퍼에 부착 — 이후 팔과 함께 움직이고 충돌 계산에 포함된다."""
        aco = AttachedCollisionObject()
        aco.link_name = ATTACH_LINK
        aco.object.id = name
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = TOUCH_LINKS

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)

        ok = self._call(self.scene_cli, ApplyPlanningScene.Request(scene=scene))
        self.get_logger().info(f"[attach] {name} {'성공' if ok else '실패'}")
        return bool(ok)

    def detach(self, name):
        aco = AttachedCollisionObject()
        aco.link_name = ATTACH_LINK
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)

        ok = self._call(self.scene_cli, ApplyPlanningScene.Request(scene=scene))
        self.get_logger().info(f"[detach] {name} {'성공' if ok else '실패'}")
        return bool(ok)

    # ── 팔 이동 ─────────────────────────────────────────────
    def move_to_pose(self, x, y, z, quat=DOWN_Q, pos_tol=0.01, ang_tol=0.1):
        """TCP 를 지정 좌표·자세로 이동 (플래너가 경로를 스스로 찾는다)."""
        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = TCP_LINK
        pc.target_point_offset = Vector3()
        pc.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])
        ]
        pc.constraint_region.primitive_poses = [make_pose(x, y, z, (0.0, 0.0, 0.0, 1.0))]
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = BASE_FRAME
        oc.link_name = TCP_LINK
        oc.orientation = make_pose(0, 0, 0, quat).orientation
        oc.absolute_x_axis_tolerance = ang_tol
        oc.absolute_y_axis_tolerance = ang_tol
        oc.absolute_z_axis_tolerance = ang_tol
        oc.weight = 1.0

        goal = MoveGroup.Goal()
        goal.request.group_name = ARM_GROUP
        goal.request.goal_constraints = [
            Constraints(position_constraints=[pc], orientation_constraints=[oc])
        ]
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = VEL_SCALE
        goal.request.max_acceleration_scaling_factor = ACC_SCALE
        goal.planning_options.plan_only = False

        res = self._send_goal(self.move_cli, goal)
        ok = res is not None and res.error_code.val == 1
        self.get_logger().info(
            f"[arm] ({x:.3f}, {y:.3f}, {z:.3f}) → {'성공' if ok else '실패'}")
        return ok

    def move_linear(self, x, y, z, quat=DOWN_Q):
        """직선(직교) 이동 — 파지 직전 접근·후퇴에 사용."""
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.group_name = ARM_GROUP
        req.link_name = TCP_LINK
        req.waypoints = [make_pose(x, y, z, quat)]
        req.max_step = 0.005
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE

        res = self._call(self.cart_cli, req)
        if res is None or res.fraction < 0.9:
            frac = 0.0 if res is None else res.fraction
            self.get_logger().warn(
                f"[arm] 직선 경로 {frac*100:.0f}% 만 생성됨 → 일반 플래닝으로 대체")
            return self.move_to_pose(x, y, z, quat)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        out = self._send_goal(self.exec_cli, goal)
        ok = out is not None and out.error_code.val == 1
        self.get_logger().info(
            f"[arm] 직선 이동 ({x:.3f}, {y:.3f}, {z:.3f}) → {'성공' if ok else '실패'}")
        return ok

    # ── 그리퍼 ──────────────────────────────────────────────
    def move_gripper(self, value):
        jc = JointConstraint()
        jc.joint_name = GRIPPER_JOINT
        jc.position = float(value)
        jc.tolerance_above = 0.02
        jc.tolerance_below = 0.02
        jc.weight = 1.0

        goal = MoveGroup.Goal()
        goal.request.group_name = GRIPPER_GROUP
        goal.request.goal_constraints = [Constraints(joint_constraints=[jc])]
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 2.0
        goal.request.max_velocity_scaling_factor = VEL_SCALE
        goal.request.max_acceleration_scaling_factor = ACC_SCALE
        goal.planning_options.plan_only = False

        res = self._send_goal(self.move_cli, goal)
        ok = res is not None and res.error_code.val == 1
        self.get_logger().info(f"[grip] {value:.2f} rad → {'성공' if ok else '실패'}")
        return ok

    # ── 시퀀스 ──────────────────────────────────────────────
    def pick(self, name, x, y, z):
        """(x, y, z) 에 있는 물체를 집는다. z 는 물체 중심 높이."""
        self.get_logger().info(f"===== PICK {name} =====")
        steps = [
            ("접근 자세",   lambda: self.move_to_pose(x, y, z + APPROACH_HEIGHT)),
            ("그리퍼 열기", lambda: self.move_gripper(GRIPPER_OPEN)),
            ("하강",       lambda: self.move_linear(x, y, z)),
            # 닫기 "전"에 부착 — 접촉이 생기기 전에 touch_links 허가증을 발급한다
            ("부착",       lambda: self.attach(name)),
            ("그리퍼 닫기", lambda: self.move_gripper(GRIPPER_CLOSED)),
            ("상승",       lambda: self.move_linear(x, y, z + APPROACH_HEIGHT)),
        ]
        return self._run(steps)

    def place(self, name, x, y, z):
        self.get_logger().info(f"===== PLACE {name} =====")
        steps = [
            ("이송",       lambda: self.move_to_pose(x, y, z + APPROACH_HEIGHT)),
            ("하강",       lambda: self.move_linear(x, y, z)),
            # attach 상태에서 열어야 touch_links 보호가 살아 있다
            ("그리퍼 열기", lambda: self.move_gripper(GRIPPER_OPEN)),
            ("살짝 후퇴",   lambda: self.move_linear(x, y, z + RETREAT_LIFT)),
            # 손가락이 물체에서 완전히 빠진 뒤 분리
            ("분리",       lambda: self.detach(name)),
            ("후퇴",       lambda: self.move_linear(x, y, z + APPROACH_HEIGHT)),
        ]
        return self._run(steps)

    def _run(self, steps):
        for label, fn in steps:
            self.get_logger().info(f"--- {label}")
            if not fn():
                self.get_logger().error(f"'{label}' 실패 — 시퀀스를 중단합니다")
                return False
        return True


def main():
    rclpy.init()
    node = PickPlace()

    # ── 미션 정의 ────────────────────────────────────────────
    BLOCK = 0.05                       # 블록 한 변 [m]
    TABLE_Z = 0.0                      # 테이블 상판 높이
    PICK_XY = (0.35, 0.15)             # 집는 위치
    PLACE_XY = (0.35, -0.15)           # 놓는 위치

    grasp_z = TABLE_Z + BLOCK / 2      # 블록 중심

    # 씬 구성
    node.add_box("table", (0.5, 0.8, 0.02), (0.5, 0.0, TABLE_Z - 0.011))
    node.add_box("block", (BLOCK,) * 3, (*PICK_XY, grasp_z))

    ok = node.pick("block", *PICK_XY, grasp_z)
    if ok:
        ok = node.place("block", *PLACE_XY, grasp_z)

    node.get_logger().info("완료" if ok else "실패로 종료")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
