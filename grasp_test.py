#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""grasp_test.py — 파지 검증 하네스 (RM75 + Inspire EG2-4C2, 시뮬레이션)

목적
    "이 위치에, 이 크기의, 이 모양 타겟이 있을 때 잡는 모션이 잘 나오는가"를
    비전 없이 좌표 입력만으로 검증한다. 나중에 카메라(YOLO+뎁스)가 좌표를 주면
    run 호출부만 바꾸면 된다.

준비
    ros2 launch rm_75_jaw_config demo.launch.py     # 터미널 1

사용 예
    # 기본: (0.35, 0.15, 테이블 위) 지름 6cm 높이 18cm 원기둥(물통)
    python3 grasp_test.py

    # 위치·크기 지정
    python3 grasp_test.py --shape cylinder --pos 0.40 0.10 --radius 0.03 --height 0.18

    # 박스
    python3 grasp_test.py --shape box --pos 0.35 0.15 --size 0.05 0.05 0.10

    # 놓는 위치까지 지정 (기본은 y 부호 반전)
    python3 grasp_test.py --pos 0.35 0.15 --place 0.30 -0.20

    # 픽만 하고 이송·놓기는 생략 (파지 검증만 빠르게)
    python3 grasp_test.py --pick-only

    # 낮은 테이블 위 먼 물체를 측면에서 감싸쥠
    python3 grasp_test.py --shape box --pos 0.5459 0.0 --size 0.055 0.055 0.21 \
        --table-z -0.115 --grasp-dir front

좌표계
    모든 좌표는 base_link(로봇 베이스) 기준 [m].
    --pos 는 물체 바닥 중심의 (x, y). z 는 테이블 높이에서 자동 계산.
    파지점은 물체 종류에 따라:
        cylinder / box → 몸통 중앙 높이 (옆에서 감싸쥠)
        sphere         → 구 중심
"""
import argparse
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
# 로봇 설정 (rm_75_jaw_config 기준 — 다른 설정이면 여기만 수정)
# ─────────────────────────────────────────────────────────────
BASE_FRAME = "base_link"
ARM_GROUP = "rm_group"
GRIPPER_GROUP = "gripper"
TCP_LINK = "grasp_tcp"
ATTACH_LINK = "Link7"

GRIPPER_JOINT = "jaw_Joint1"
GRIPPER_OPEN = 0.80
GRIPPER_MAX_WIDTH = 0.080       # 완전 개방 시 두 패드 사이 폭 [m] (근사)

DOWN_Q = (1.0, 0.0, 0.0, 0.0)   # 그리퍼가 -z(바닥)를 향함 (위에서 내려찍기)
FRONT_Q = (0.5, 0.5, 0.5, 0.5)  # 그리퍼가 +x(전방)를 향하고 손가락은 수평(y) 개폐
                                # (공구 z→월드 x, 공구 x→월드 y 가 되는 회전)

APPROACH_HEIGHT = 0.12
RETREAT_LIFT = 0.06             # place 후 detach 전 이탈 높이
VEL_SCALE = 0.1
ACC_SCALE = 0.1

TOUCH_LINKS = [
    "Link5", "Link6",
    "Link7", "4C2_baselink",
    "4C2_Link1", "4C2_Link2", "4C2_Link3",
    "4C2_Link4", "4C2_Link5", "4C2_Link6",
    "grasp_tcp",
]

TABLE_TOP_Z = 0.0               # 물체가 놓인 면의 높이 (base_link 기준, --table-z 로 지정)


def make_pose(x, y, z, quat=DOWN_Q):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = \
        (float(v) for v in quat)
    return p


def closed_value_for_width(width):
    """물체 폭 → jaw_Joint1 닫힘값 근사.

    개폐폭이 각도에 대략 선형이라고 가정:
        joint=0.80 → 폭 GRIPPER_MAX_WIDTH,  joint=0 → 폭 0
    물체를 살짝 조이도록 5mm 여유를 뺀 폭으로 계산한다.
    시뮬(충돌 기반)에서는 '물체 폭보다 약간 큰 값'이면 충분하다.
    """
    grip_w = max(0.0, width - 0.005)
    v = GRIPPER_OPEN * grip_w / GRIPPER_MAX_WIDTH
    return min(max(v, 0.05), GRIPPER_OPEN - 0.05)


class GraspTest(Node):
    def __init__(self):
        super().__init__("grasp_test")
        self.move_cli = ActionClient(self, MoveGroup, "/move_action")
        self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")

        for name, cli in (("/move_action", self.move_cli),
                          ("/execute_trajectory", self.exec_cli)):
            if not cli.wait_for_server(timeout_sec=10.0):
                raise SystemExit(f"{name} 없음 — demo.launch.py 먼저.")
        for name, cli in (("/apply_planning_scene", self.scene_cli),
                          ("/compute_cartesian_path", self.cart_cli)):
            if not cli.wait_for_service(timeout_sec=10.0):
                raise SystemExit(f"{name} 없음 — demo.launch.py 먼저.")
        self.get_logger().info("MoveIt 연결 완료")

    # ── 공통 ────────────────────────────────────────────────
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

    def _apply(self, scene):
        return bool(self._call(self.scene_cli, ApplyPlanningScene.Request(scene=scene)))

    # ── Planning Scene ──────────────────────────────────────
    def spawn_target(self, name, shape, center_xyz, dims):
        """타겟 도형 등록.

        shape='box'      dims=(sx, sy, sz)
        shape='cylinder' dims=(height, radius)
        shape='sphere'   dims=(radius,)
        center_xyz 는 도형의 기하 중심.
        """
        prim = SolidPrimitive()
        if shape == "box":
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [float(d) for d in dims]
        elif shape == "cylinder":
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [float(dims[0]), float(dims[1])]  # [height, radius]
        elif shape == "sphere":
            prim.type = SolidPrimitive.SPHERE
            prim.dimensions = [float(dims[0])]
        else:
            raise ValueError(f"unknown shape: {shape}")

        co = CollisionObject()
        co.header.frame_id = BASE_FRAME
        co.id = name
        co.primitives = [prim]
        co.primitive_poses = [make_pose(*center_xyz, quat=(0.0, 0.0, 0.0, 1.0))]
        co.operation = CollisionObject.ADD

        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(co)
        ok = self._apply(scene)
        self.get_logger().info(
            f"[scene] {name} ({shape}, dims={dims}) @ "
            f"({center_xyz[0]:.3f}, {center_xyz[1]:.3f}, {center_xyz[2]:.3f}) "
            f"→ {'성공' if ok else '실패'}")
        return ok

    def remove_object(self, name):
        co = CollisionObject()
        co.header.frame_id = BASE_FRAME
        co.id = name
        co.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(co)
        return self._apply(scene)

    def attach(self, name):
        aco = AttachedCollisionObject()
        aco.link_name = ATTACH_LINK
        aco.object.id = name
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = TOUCH_LINKS
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        ok = self._apply(scene)
        self.get_logger().info(f"[attach] {name} → {'성공' if ok else '실패'}")
        return ok

    def detach(self, name):
        aco = AttachedCollisionObject()
        aco.link_name = ATTACH_LINK
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        ok = self._apply(scene)
        self.get_logger().info(f"[detach] {name} → {'성공' if ok else '실패'}")
        return ok

    # ── 팔 ──────────────────────────────────────────────────
    def move_to_pose(self, x, y, z, quat=DOWN_Q, pos_tol=0.01, ang_tol=0.1):
        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = TCP_LINK
        pc.target_point_offset = Vector3()
        pc.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])]
        pc.constraint_region.primitive_poses = [
            make_pose(x, y, z, (0.0, 0.0, 0.0, 1.0))]
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
            Constraints(position_constraints=[pc], orientation_constraints=[oc])]
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = VEL_SCALE
        goal.request.max_acceleration_scaling_factor = ACC_SCALE
        goal.planning_options.plan_only = False

        res = self._send_goal(self.move_cli, goal)
        ok = res is not None and res.error_code.val == 1
        self.get_logger().info(
            f"[arm] pose ({x:.3f}, {y:.3f}, {z:.3f}) → {'성공' if ok else '실패'}")
        return ok

    def move_linear(self, x, y, z, quat=DOWN_Q):
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
                f"[arm] 직선 {frac*100:.0f}% → 일반 플래닝 대체")
            return self.move_to_pose(x, y, z, quat)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        out = self._send_goal(self.exec_cli, goal)
        ok = out is not None and out.error_code.val == 1
        self.get_logger().info(
            f"[arm] linear ({x:.3f}, {y:.3f}, {z:.3f}) → {'성공' if ok else '실패'}")
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
    def _run(self, steps):
        for label, fn in steps:
            self.get_logger().info(f"--- {label}")
            if not fn():
                self.get_logger().error(f"'{label}' 실패 — 중단")
                return False
        return True

    def pick(self, name, gx, gy, gz, closed_value, direction="top"):
        """(gx, gy, gz) = 파지점(TCP가 갈 지점).

        direction='top'   : 위에서 하강 (DOWN_Q)
        direction='front' : 뒤(-x)에서 전진해 측면 감싸쥠 (FRONT_Q)
                            물통처럼 긴 물체, 먼 거리 타겟에 적합
        """
        if direction == "front":
            q = FRONT_Q
            depth_off = 0.015                              # 손바닥 침범 방지
            gx_in = gx - depth_off                         # 실제 진입 목표
            pre  = (gx - APPROACH_HEIGHT, gy, gz)
            lift = (gx_in, gy, gz + RETREAT_LIFT)
            steps = [
                ("접근",    lambda: self.move_to_pose(*pre, quat=q)),
                ("열기",    lambda: self.move_gripper(GRIPPER_OPEN)),
                ("부착",    lambda: self.attach(name)),       # 진입 전에!
                ("진입",    lambda: self.move_linear(gx_in, gy, gz, quat=q)),
                ("닫기",    lambda: self.move_gripper(closed_value)),
                ("들어올림", lambda: self.move_linear(*lift, quat=q)),
            ]
        else:
            q = DOWN_Q
            pre  = (gx, gy, gz + APPROACH_HEIGHT)
            lift = (gx, gy, gz + APPROACH_HEIGHT)
            steps = [
                ("접근",    lambda: self.move_to_pose(*pre, quat=q)),
                ("열기",    lambda: self.move_gripper(GRIPPER_OPEN)),
                ("진입",    lambda: self.move_linear(gx, gy, gz, quat=q)),
                ("부착",    lambda: self.attach(name)),       # 닫기 전에!
                ("닫기",    lambda: self.move_gripper(closed_value)),
                ("들어올림", lambda: self.move_linear(*lift, quat=q)),
            ]
        self.get_logger().info(
            f"===== PICK {name} @ ({gx:.3f},{gy:.3f},{gz:.3f}) [{direction}] =====")
        return self._run(steps)

    def place(self, name, gx, gy, gz, respawn=None, direction="top"):
        if direction == "front":
            q = FRONT_Q
            exit_pose = (gx - APPROACH_HEIGHT, gy, gz)     # 뒤로 빠짐
        else:
            q = DOWN_Q
            exit_pose = (gx, gy, gz + RETREAT_LIFT)        # 위로 빠짐
        steps = [
            ("이송",       lambda: self.move_to_pose(gx, gy, gz + RETREAT_LIFT, quat=q)),
            ("하강",       lambda: self.move_linear(gx, gy, gz, quat=q)),
            ("열기",       lambda: self.move_gripper(GRIPPER_OPEN)),   # attach 상태에서!
            ("이탈",       lambda: self.move_linear(*exit_pose, quat=q)),
            ("분리",       lambda: self.detach(name)),
        ]
        if respawn is not None:
            shape, center, dims = respawn
            steps.append(("재배치", lambda: (self.remove_object(name) or True)
                                    and self.spawn_target(name, shape, center, dims)))
        if direction == "front":
            steps.append(("후퇴", lambda: self.move_linear(
                gx - APPROACH_HEIGHT, gy, gz + RETREAT_LIFT, quat=q)))
        else:
            steps.append(("후퇴", lambda: self.move_linear(
                gx, gy, gz + APPROACH_HEIGHT, quat=q)))
        self.get_logger().info(
            f"===== PLACE {name} @ ({gx:.3f},{gy:.3f},{gz:.3f}) [{direction}] =====")
        return self._run(steps)


# ─────────────────────────────────────────────────────────────
# 타겟 정의 → 파지점 계산
# ─────────────────────────────────────────────────────────────
TOP_GRASP_DEPTH = 0.01          # top 파지 시 물체 윗면에서 내려가는 깊이 [m]


def build_target(args):
    """CLI 인자 → (shape, dims, center_z, grasp_z, grasp_width)"""
    top = (args.grasp_dir == "top")
    depth = args.grasp_depth

    if args.shape == "cylinder":
        h, r = args.height, args.radius
        center_z = TABLE_TOP_Z + h / 2
        grasp_z = max(TABLE_TOP_Z + h - depth, center_z) if top else center_z
        return "cylinder", (h, r), center_z, grasp_z, 2 * r

    if args.shape == "sphere":
        r = args.radius
        center_z = TABLE_TOP_Z + r
        return "sphere", (r,), center_z, center_z, 2 * r

    # box
    sx, sy, sz = args.size
    center_z = TABLE_TOP_Z + sz / 2
    grasp_z = max(TABLE_TOP_Z + sz - depth, center_z) if top else center_z
    grasp_w = sx if top else sy     # front는 손가락이 y로 개폐됨
    return "box", (sx, sy, sz), center_z, grasp_z, grasp_w

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--shape", choices=["box", "cylinder", "sphere"],
                    default="cylinder")
    ap.add_argument("--pos", nargs=2, type=float, metavar=("X", "Y"),
                    default=[0.35, 0.15], help="물체 바닥 중심 (base_link 기준)")
    ap.add_argument("--place", nargs=2, type=float, metavar=("X", "Y"),
                    default=None, help="놓는 위치 (기본: y 부호 반전)")
    ap.add_argument("--radius", type=float, default=0.03,
                    help="cylinder/sphere 반지름 [m]")
    ap.add_argument("--height", type=float, default=0.18,
                    help="cylinder 높이 [m]")
    ap.add_argument("--size", nargs=3, type=float, metavar=("SX", "SY", "SZ"),
                    default=[0.05, 0.05, 0.10], help="box 크기 [m]")
    ap.add_argument("--pick-only", action="store_true",
                    help="파지·상승까지만 (이송/놓기 생략)")
    ap.add_argument("--no-table", action="store_true",
                    help="테이블 미등록")
    ap.add_argument("--grasp-dir", choices=["top", "front"], default="top",
                    help="top=위에서 하강 / front=측면 감싸쥠 (먼 타겟용)")
    ap.add_argument("--table-z", type=float, default=0.0,
                    help="물체가 놓인 면의 높이 [m] (base_link 기준, 아래면 음수)")
    ap.add_argument("--grasp-depth", type=float, default=TOP_GRASP_DEPTH,
                    help="top 파지 시 물체 윗면에서 내려가는 깊이 [m]")
    args = ap.parse_args()
    
    global TABLE_TOP_Z
    TABLE_TOP_Z = args.table_z

    shape, dims, center_z, grasp_z, grasp_w = build_target(args)
    closed = closed_value_for_width(grasp_w)

    if grasp_w > GRIPPER_MAX_WIDTH:
        print(f"⚠️  파지 폭 {grasp_w*1000:.0f}mm > 개폐 한계 "
              f"{GRIPPER_MAX_WIDTH*1000:.0f}mm — 잡을 수 없는 크기입니다.")
        return 1

    px, py = args.pos
    if args.place is None:
        qx, qy = px, -py
    else:
        qx, qy = args.place

    rclpy.init()
    node = GraspTest()

    if not args.no_table:
        node.spawn_target("table", "box",
                          (0.50, 0.0, TABLE_TOP_Z - 0.012), (0.5, 0.8, 0.02))

    node.spawn_target("target", shape, (px, py, center_z), dims)

    node.get_logger().info(
        f"파지 폭 {grasp_w*1000:.0f}mm → 닫힘값 {closed:.2f} rad")

    ok = node.pick("target", px, py, grasp_z, closed, direction=args.grasp_dir)
    if ok and not args.pick_only:
        respawn = (shape, (qx, qy, center_z), dims)
        ok = node.place("target", qx, qy, grasp_z,
                        respawn=respawn, direction=args.grasp_dir)

    node.get_logger().info("✅ 완료" if ok else "❌ 실패로 종료")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
