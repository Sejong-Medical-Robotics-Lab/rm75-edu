"""rm75_edu.arm_client — MoveIt2 를 부르는 이번 주의 창구.

교재 5.1의 기본 패턴(목표 설정 → plan → 확인 → execute)을 그대로 클래스로
옮긴 것이다. 내부적으로 MoveIt2 의 표준 액션 두 개를 쓴다:

    /move_action          (moveit_msgs/action/MoveGroup)      — plan_only=True 로 '계산만'
    /execute_trajectory   (moveit_msgs/action/ExecuteTrajectory) — 검증된 궤적의 '실행'

즉 plan 과 execute 가 코드 수준에서도 분리되어 있다 — RViz 의 Plan/Execute
버튼과 정확히 같은 구조다(교재 4.5). ros2 action list 에서 이 두 액션을
직접 확인해 보라(미션 A ⑤).

사용:
    arm = ArmClient()
    ok, traj = arm.plan_named("home")
    if ok:
        arm.execute(traj)          # 성공한 계획만 실행 (교재 5.1의 원칙)
"""
from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, OrientationConstraint,
                             PositionConstraint)
from shape_msgs.msg import SolidPrimitive

from .poses import ARM_GROUP, ARM_JOINTS, DOWN_Q, HOME, ZERO

NAMED = {"home": HOME, "zero": ZERO}


class ArmClient:
    def __init__(self, node: Node | None = None, group: str = ARM_GROUP):
        self._own = node is None
        if self._own:
            if not rclpy.ok():
                rclpy.init()
            node = rclpy.create_node("arm_client")
        self.node = node
        self.group = group
        self._move = ActionClient(node, MoveGroup, "/move_action")
        self._exec = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
        for cli, name in ((self._move, "/move_action"),
                          (self._exec, "/execute_trajectory")):
            if not cli.wait_for_server(timeout_sec=5.0):
                raise RuntimeError(
                    f"{name} 액션 서버가 없습니다 — demo 모드(런치)가 먼저 떠 "
                    "있어야 합니다 (README 빠른 시작).")

    # ---- 계획 (계산만 — 아직 움직이지 않는다) --------------------------
    def _plan(self, constraints: Constraints):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 3
        goal.request.max_velocity_scaling_factor = 0.1     # 교재 5.4: 속도 욕심 금지
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True             # ← '계산만'의 스위치
        fut = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, None, "MoveGroup 목표가 거부되었습니다"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        res = rf.result().result
        if res.error_code.val != 1:      # 1 = SUCCESS
            return False, None, f"Plan 실패 (MoveItErrorCodes={res.error_code.val})"
        return True, res.planned_trajectory, \
            f"Plan 성공 (계획 시간 {res.planning_time:.2f}s)"

    def plan_pose(self, x, y, z, quat=DOWN_Q, frame="base_link"):
        """pose goal — 손끝(TCP)의 위치 + 자세 (교재 5.1의 기본 문형)."""
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = \
            float(x), float(y), float(z)
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = [float(v) for v in quat]

        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = "gripper_tcp"
        box = SolidPrimitive(type=SolidPrimitive.BOX,
                             dimensions=[0.005, 0.005, 0.005])
        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(ps.pose)
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = "gripper_tcp"
        oc.orientation = ps.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        return self._plan(c)

    def plan_joints(self, q):
        """joint goal — 일곱 관절 각도를 직접 지정 (IK 불필요, 항상 유일)."""
        c = Constraints()
        for name, val in zip(ARM_JOINTS, q):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = jc.tolerance_below = 0.005
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return self._plan(c)

    def plan_named(self, name: str):
        """named target — 'home' 처럼 이름 붙은 자세 (SRDF 와 동일 값)."""
        if name not in NAMED:
            return False, None, f"알 수 없는 자세 이름: {name} (가능: {list(NAMED)})"
        return self.plan_joints(NAMED[name])

    # ---- 실행 (검증된 궤적만) ------------------------------------------
    def execute(self, traj) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        fut = self._exec.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        return rf.result().result.error_code.val == 1

    # ---- 편의: 계획 → 확인 → 실행을 한 번에 (내부에 ⑤가 강제되어 있다) --
    def go_pose(self, x, y, z, quat=DOWN_Q) -> bool:
        ok, traj, msg = self.plan_pose(x, y, z, quat)
        print(msg)
        if not ok:
            return False                 # 실패한 계획은 실행하지 않는다 (5.1)
        return self.execute(traj)

    def go_named(self, name: str) -> bool:
        ok, traj, msg = self.plan_named(name)
        print(msg)
        if not ok:
            return False
        return self.execute(traj)

    def shutdown(self):
        if self._own:
            self.node.destroy_node()


def wait(sec: float):
    time.sleep(sec)
