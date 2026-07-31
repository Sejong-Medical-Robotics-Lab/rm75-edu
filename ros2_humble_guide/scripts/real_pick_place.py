#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""real_pick_place.py — RM75 실기체 픽앤플레이스
(팔: MoveIt2 계획+충돌검사 / 그리퍼: rm_driver 토픽 힘 제어)

준비:
    터미널 1: ros2 launch rm_bringup rm_75_bringup.launch.py
    터미널 2: source ~/ros2_ws/install/setup.bash
              python3 real_pick_place.py --approach-only   # 첫 시도: A 위 접근 자세까지만
              python3 real_pick_place.py                   # 정렬 확인 후 전체 시퀀스
              python3 real_pick_place.py --auto            # 단계별 Enter 확인 생략(충분히 검증된 뒤에만)

안전 설계 (SOP ④를 코드로 강제):
    - 모든 팔 이동은  Plan(계산) → RViz 잔상 확인 → Enter → Execute  순서로만 진행된다.
    - Enter 대신 q 를 입력하면 즉시 중단된다.
    - 실행 전 반드시: 비상정지 전담 위치 확인 · 작업 공간 정리 · 속도 스케일 0.1 유지.

사용 전 반드시 아래 [설정] 블록을 내 실측값으로 수정할 것 (03_practice 6-2절).
"""
import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool

from rm_ros_interfaces.msg import Gripperpick, Gripperset

# ─────────────────────────────────────────────────────────────
# [설정] ★ 반드시 실측값으로 수정 ★
# ─────────────────────────────────────────────────────────────
# ① 홈 자세 — 03_practice 5-2절에서 기록한 /joint_states 값 7개 (라디안)
HOME_JOINTS = [0.0, -0.60, 0.0, 1.16, 0.0, 1.89, 0.0]

# ② 테이블/그리퍼 실측 — 03_practice 6-2절
TABLE_HEIGHT = -0.115   # base_link 기준 테이블 면 z [m] (테이블이 베이스보다 낮으면 음수)
FINGER_OFFSET = 0.12    # Link7 플랜지 → 손가락 파지 중심 거리 [m]

# ③ 물체와 위치 — 자로 측정 (base_link 기준, 단위 m! / x 전방+, y 왼쪽+)
PICK_XY = (0.45, 0.10)    # 집을 물체의 바닥 중심
PLACE_XY = (0.45, -0.10)  # 내려놓을 위치
OBJ_H = 0.18              # 물체 높이 [m] — 파지점은 몸통 중앙 (TABLE_HEIGHT + OBJ_H/2)

# ④ 그리퍼 값 — force 는 이분 탐색으로 튜닝 (놓치면 ↑, 찌그러지면 ↓)
GRIPPER_OPEN_POS = 1000   # 1~1000 (1000 = 최대 개방)
GRIPPER_SPEED = 200
GRIPPER_FORCE = 300
GRIPPER_TIMEOUT = 10      # s

# ─────────────────────────────────────────────────────────────
# 고정 설정 (rm_75_bringup 기본 구성 기준 — 보통 수정 불필요)
# ─────────────────────────────────────────────────────────────
BASE_FRAME = "base_link"
ARM_GROUP = "rm_group"
EE_LINK = "Link7"                # bringup 기본 모델은 그리퍼 미포함 → 플랜지가 말단
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4",
               "joint5", "joint6", "joint7"]

DOWN_Q = (1.0, 0.0, 0.0, 0.0)    # 그리퍼가 바닥을 향하는 자세 (x축 180°)
APPROACH = 0.12                  # 접근/후퇴 높이 [m]
VEL_SCALE = 0.1                  # 실기체는 0.1 고정. 올리지 말 것.
ACC_SCALE = 0.1

WORK_R_MIN, WORK_R_MAX = 0.20, 0.65   # 파지점 허용 반경 [m] — 벗어나면 실행 전에 거부


def make_pose(x, y, z, quat=DOWN_Q):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = \
        (float(v) for v in quat)
    return p


class RealPickPlace(Node):
    def __init__(self, auto=False):
        super().__init__("real_pick_place")
        self.auto = auto

        self.move_cli = ActionClient(self, MoveGroup, "/move_action")
        self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")

        for name, cli in (("/move_action", self.move_cli),
                          ("/execute_trajectory", self.exec_cli)):
            if not cli.wait_for_server(timeout_sec=10.0):
                raise SystemExit(f"{name} 액션 서버 없음 — rm_75_bringup 을 먼저 띄우세요.")
        for name, cli in (("/apply_planning_scene", self.scene_cli),
                          ("/compute_cartesian_path", self.cart_cli)):
            if not cli.wait_for_service(timeout_sec=10.0):
                raise SystemExit(f"{name} 서비스 없음 — rm_75_bringup 을 먼저 띄우세요.")

        # 그리퍼: 드라이버 토픽 (cmd 발행 → result 회신 대기)
        self.grip_pos_pub = self.create_publisher(
            Gripperset, "/rm_driver/set_gripper_position_cmd", 10)
        self.grip_pick_pub = self.create_publisher(
            Gripperpick, "/rm_driver/set_gripper_pick_on_cmd", 10)
        self._grip_result = None
        self.create_subscription(
            Bool, "/rm_driver/set_gripper_position_result", self._on_grip, 10)
        self.create_subscription(
            Bool, "/rm_driver/set_gripper_pick_on_result", self._on_grip, 10)

        self.get_logger().info("MoveIt · 드라이버 연결 완료")

    # ── 공통 유틸 ───────────────────────────────────────────
    def _call(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def _send_goal(self, client, goal):
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        h = fut.result()
        if h is None or not h.accepted:
            return None
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result

    def confirm(self, label):
        """RViz 잔상 확인 게이트. auto 모드가 아니면 Enter를 기다린다."""
        if self.auto:
            return True
        ans = input(f"  ▶ [{label}] RViz 잔상을 확인했으면 Enter, 중단은 q+Enter: ").strip()
        if ans.lower() == "q":
            self.get_logger().warn("사용자 중단")
            return False
        return True

    def _execute(self, trajectory, label):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        res = self._send_goal(self.exec_cli, goal)
        ok = res is not None and res.error_code.val == 1
        self.get_logger().info(f"[실행] {label} → {'성공' if ok else '실패'}")
        return ok

    # ── 팔: Plan(잔상) → confirm → Execute ─────────────────
    def move_joints(self, joints, label="관절 목표"):
        goal = MoveGroup.Goal()
        goal.request.group_name = ARM_GROUP
        cons = Constraints()
        for n, v in zip(JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(v)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            cons.joint_constraints.append(jc)
        goal.request.goal_constraints = [cons]
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = VEL_SCALE
        goal.request.max_acceleration_scaling_factor = ACC_SCALE
        goal.planning_options.plan_only = True          # ← 계획만! (잔상 표시)

        res = self._send_goal(self.move_cli, goal)
        if res is None or res.error_code.val != 1:
            self.get_logger().error(f"[계획] {label} 실패")
            return False
        self.get_logger().info(f"[계획] {label} 성공 — RViz 잔상 확인")
        if not self.confirm(label):
            return False
        return self._execute(res.planned_trajectory, label)

    def move_pose(self, x, y, z, quat=DOWN_Q, label=None,
                  pos_tol=0.01, ang_tol=0.1):
        label = label or f"pose ({x:.3f}, {y:.3f}, {z:.3f})"
        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = EE_LINK
        pc.target_point_offset = Vector3()
        pc.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])]
        pc.constraint_region.primitive_poses = [
            make_pose(x, y, z, (0.0, 0.0, 0.0, 1.0))]
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = BASE_FRAME
        oc.link_name = EE_LINK
        oc.orientation = make_pose(0, 0, 0, quat).orientation
        oc.absolute_x_axis_tolerance = ang_tol
        oc.absolute_y_axis_tolerance = ang_tol
        oc.absolute_z_axis_tolerance = ang_tol
        oc.weight = 1.0

        goal = MoveGroup.Goal()
        goal.request.group_name = ARM_GROUP
        goal.request.goal_constraints = [
            Constraints(position_constraints=[pc], orientation_constraints=[oc])]
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = VEL_SCALE
        goal.request.max_acceleration_scaling_factor = ACC_SCALE
        goal.planning_options.plan_only = True          # ← 계획만! (잔상 표시)

        res = self._send_goal(self.move_cli, goal)
        if res is None or res.error_code.val != 1:
            self.get_logger().error(f"[계획] {label} 실패 "
                                    "(도달·자세·충돌 세 갈래를 점검하세요)")
            return False
        self.get_logger().info(f"[계획] {label} 성공 — RViz 잔상 확인")
        if not self.confirm(label):
            return False
        return self._execute(res.planned_trajectory, label)

    def move_linear(self, x, y, z, quat=DOWN_Q, label=None):
        """직선(직교) 이동 — 하강/상승에 사용. 계획 후 confirm을 거친다."""
        label = label or f"직선 ({x:.3f}, {y:.3f}, {z:.3f})"
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.group_name = ARM_GROUP
        req.link_name = EE_LINK
        req.waypoints = [make_pose(x, y, z, quat)]
        req.max_step = 0.005
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE

        res = self._call(self.cart_cli, req)
        frac = 0.0 if res is None else res.fraction
        if frac < 0.9:
            self.get_logger().warn(f"[계획] {label}: 직선 {frac*100:.0f}% 만 생성 "
                                   "→ 일반 플래닝으로 대체 시도")
            return self.move_pose(x, y, z, quat, label=label + " (대체)")

        self.get_logger().info(f"[계획] {label} 직선 {frac*100:.0f}% — RViz 잔상 확인")
        if not self.confirm(label):
            return False
        return self._execute(res.solution, label)

    # ── Planning Scene ──────────────────────────────────────
    def add_table(self):
        co = CollisionObject()
        co.header.frame_id = BASE_FRAME
        co.id = "table"
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                        dimensions=[1.2, 1.2, 0.02])]
        co.primitive_poses = [make_pose(0.45, 0.0, TABLE_HEIGHT - 0.011,
                                        quat=(0.0, 0.0, 0.0, 1.0))]
        co.operation = CollisionObject.ADD
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(co)
        ok = self._call(self.scene_cli, ApplyPlanningScene.Request(scene=scene))
        self.get_logger().info(f"[scene] 테이블 등록 {'성공' if ok else '실패'}")
        return bool(ok)

    # ── 그리퍼 (rm_driver 토픽) ─────────────────────────────
    def _on_grip(self, msg: Bool):
        self._grip_result = bool(msg.data)

    def _grip_wait(self):
        end = time.time() + GRIPPER_TIMEOUT + 3.0
        while time.time() < end and self._grip_result is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._grip_result is None:
            self.get_logger().warn("그리퍼 응답 시간 초과 (계속 진행)")
        elif not self._grip_result:
            self.get_logger().warn("그리퍼가 실패를 보고했습니다 (계속 진행)")
        return True

    @staticmethod
    def _set_if_exists(msg, field, value):
        # 드라이버 버전에 따라 없는 필드(예: timeout)는 조용히 건너뛴다
        if hasattr(msg, field):
            setattr(msg, field, value)

    def gripper_open(self):
        msg = Gripperset()
        msg.position = int(GRIPPER_OPEN_POS)
        self._set_if_exists(msg, "block", True)
        self._set_if_exists(msg, "timeout", int(GRIPPER_TIMEOUT))
        self.get_logger().info(f"그리퍼 개방 (position={msg.position})")
        self._grip_result = None
        self.grip_pos_pub.publish(msg)
        return self._grip_wait()

    def gripper_grasp(self):
        """힘 제어 파지 — 설정한 힘을 계속 유지한다 (이송 중 미끄러짐 방지)."""
        msg = Gripperpick()
        msg.speed = int(GRIPPER_SPEED)
        msg.force = int(GRIPPER_FORCE)
        self._set_if_exists(msg, "block", True)
        self._set_if_exists(msg, "timeout", int(GRIPPER_TIMEOUT))
        self.get_logger().info(f"그리퍼 힘 제어 파지 (force={msg.force})")
        self._grip_result = None
        self.grip_pick_pub.publish(msg)
        return self._grip_wait()

    # ── 시퀀스 ──────────────────────────────────────────────
    def run(self, approach_only=False):
        # 파지점(물체 몸통 중앙)과 이에 대응하는 Link7(플랜지) 목표 높이
        grasp_z = TABLE_HEIGHT + OBJ_H / 2.0
        flange_grasp_z = grasp_z + FINGER_OFFSET       # 위에서 수직 파지 기준
        flange_appr_z = flange_grasp_z + APPROACH

        # 실행 전 안전 검사: 작업 반경
        for tag, (x, y) in (("PICK", PICK_XY), ("PLACE", PLACE_XY)):
            r = (x**2 + y**2) ** 0.5
            if not (WORK_R_MIN <= r <= WORK_R_MAX):
                self.get_logger().error(
                    f"{tag} 위치 반경 {r:.3f} m 가 허용 범위"
                    f"({WORK_R_MIN}~{WORK_R_MAX} m)를 벗어남 — 좌표를 다시 재세요.")
                return False

        print("─" * 60)
        print("설정 요약  (틀리면 지금 Ctrl+C)")
        print(f"  홈 관절값      : {HOME_JOINTS}")
        print(f"  테이블 높이    : {TABLE_HEIGHT:+.3f} m (base 기준)")
        print(f"  FINGER_OFFSET  : {FINGER_OFFSET:.3f} m")
        print(f"  PICK  {PICK_XY} → 파지점 z {grasp_z:+.3f} (플랜지 z {flange_grasp_z:+.3f})")
        print(f"  PLACE {PLACE_XY}")
        print(f"  파지력         : {GRIPPER_FORCE}")
        print("─" * 60)
        if not self.confirm("설정 요약"):
            return False

        px, py = PICK_XY
        qx, qy = PLACE_XY

        if not self.move_joints(HOME_JOINTS, "① 홈 자세"):
            return False
        if not self.add_table():
            return False
        self.gripper_open()

        if not self.move_pose(px, py, flange_appr_z, label="② A 위 접근 자세"):
            return False

        if approach_only:
            self.get_logger().info(
                "--approach-only 완료. 그리퍼가 물체 바로 위에 정렬됐는지 눈으로 확인하고, "
                "어긋나면 PICK_XY 를 수정한 뒤 전체 시퀀스를 실행하세요. "
                "(팔은 이 자세에 머뭅니다 — 홈 복귀는 RViz로)")
            return True

        if not self.move_linear(px, py, flange_grasp_z, label="③ 파지점으로 하강"):
            return False
        self.gripper_grasp()
        time.sleep(0.5)

        if not self.move_linear(px, py, flange_appr_z, label="④ 들어올림"):
            return False
        if not self.move_pose(qx, qy, flange_appr_z, label="⑤ B 위로 이송"):
            return False
        if not self.move_linear(qx, qy, flange_grasp_z + 0.005, label="⑥ B로 하강"):
            return False
        self.gripper_open()
        time.sleep(0.5)

        if not self.move_linear(qx, qy, flange_appr_z, label="⑦ 후퇴"):
            return False
        if not self.move_joints(HOME_JOINTS, "⑧ 홈 복귀"):
            return False

        self.get_logger().info("✅ 픽앤플레이스 완료")
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--approach-only", action="store_true",
                    help="A 위 접근 자세까지만 이동 (좌표 정렬 확인용 첫 시도)")
    ap.add_argument("--auto", action="store_true",
                    help="단계별 Enter 확인 생략 — 충분히 검증된 뒤에만 사용")
    args = ap.parse_args()

    rclpy.init()
    node = RealPickPlace(auto=args.auto)
    try:
        ok = node.run(approach_only=args.approach_only)
    except KeyboardInterrupt:
        ok = False
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
